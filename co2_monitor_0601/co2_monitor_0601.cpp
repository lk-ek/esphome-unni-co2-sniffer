// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "co2_monitor_0601.h"
#include "debug_udp.h"

#include "ble_options.h"
#include "co2_decoder.h"
#include "i2c_sniffer.h"
#include "rtrh_decoder.h"
#include "power_save.h"

#if UNNI_BLE_ENABLED
#include "sensirion_ble.h"
#include "sensirion_settings.h"
#if UNNI_SHT43_IDENTITY_PROBE
#include "sensirion_sht43_probe.h"
#endif
#endif
#if UNNI_BLE_HISTORY_ENABLED
#include "sensirion_history.h"
#endif

#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/preferences.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

namespace esphome {
namespace co2_monitor_0601 {

static const char *TAG = "co2_monitor_0601";

namespace {

inline void publish(sensor::Sensor *sensor, float value) {
  if (sensor) sensor->publish_state(value);
}

inline void publish(binary_sensor::BinarySensor *sensor, bool value) {
  if (sensor) sensor->publish_state(value);
}

inline void publish_positive(sensor::Sensor *sensor, float value) {
  if (sensor && value > 0.0f) sensor->publish_state(value);
}

inline void publish_finite(sensor::Sensor *sensor, float value) {
  if (sensor && std::isfinite(value)) sensor->publish_state(value);
}

constexpr uint32_t BATTERY_LEARNING_PREF_KEY = 0xB4172601UL;
constexpr uint16_t BATTERY_LEARNING_VERSION = 1;

struct BatteryLearningPersisted {
  uint16_t version;
  uint16_t learned_cycles;
  float learned_full_runtime_h;
  uint8_t session_active;
  uint8_t reserved[3];
  float session_start_progress;
  float session_last_progress;
  uint32_t session_elapsed_minutes;
};

uint8_t sensor_capture_in_progress_() {
  return static_cast<uint8_t>((i2c_sniffer::capture_in_progress() ? 0x01U : 0U) |
                              (rtrh_decoder::capture_in_progress() ? 0x02U : 0U));
}

}  // namespace

#if UNNI_BLE_ENABLED
void CO2Monitor0601::set_ble_advertising_interval(uint32_t interval_ms) {
  this->ble_usb_advertising_interval_ms_ = interval_ms;
  sensirion_ble_set_advertising_interval(interval_ms);
}

void CO2Monitor0601::gap_event_handler(esp_gap_ble_cb_event_t event,
                                   esp_ble_gap_cb_param_t *param) {
  sensirion_ble_gap_event_handler(event, param);
  if (param == nullptr) return;

  switch (event) {
    case ESP_GAP_BLE_SEC_REQ_EVT:
      // Existing bonds may request encryption outside the pairing window.
      // Accept the security procedure; a new MITM pairing still needs the
      // Numeric Comparison confirmation below.
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;
    case ESP_GAP_BLE_NC_REQ_EVT: {
      const uint32_t passkey = param->ble_security.key_notif.passkey;
      const bool accept = this->ble_pairing_mode_;
      ESP_LOGW(TAG, "BLE Numeric Comparison %06lu: %s",
               static_cast<unsigned long>(passkey),
               accept ? "accepted by active HA pairing window" : "rejected; pairing window is closed");
      esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, accept);
      break;
    }
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGW(TAG, "BLE pairing/authentication complete");
        if (this->ble_pairing_mode_) this->set_ble_pairing_mode(false);
      } else {
        ESP_LOGW(TAG, "BLE pairing/authentication failed, reason=0x%02X",
                 static_cast<unsigned>(param->ble_security.auth_cmpl.fail_reason));
      }
      break;
    default:
      break;
  }
}

void CO2Monitor0601::gatts_event_handler(esp_gatts_cb_event_t event,
                                     esp_gatt_if_t gatts_if,
                                     esp_ble_gatts_cb_param_t *param) {
#if UNNI_SHT43_IDENTITY_PROBE
  static bool heap_logged_after_ble_start = false;
  if (!heap_logged_after_ble_start) {
    heap_logged_after_ble_start = true;
    ESP_LOGI(TAG, "Heap after BLE stack activation: free=%u B, largest_8bit=%u B",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
  }
#endif
  sensirion_ble_gatts_event_handler(event, param);
  sensirion_settings_gatts_event_handler(event, gatts_if, param);
  if (event == ESP_GATTS_CONNECT_EVT && param != nullptr) {
    this->ble_peer_connected_ = true;
    std::memcpy(this->ble_peer_bda_, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    if (this->ble_pairing_mode_) this->begin_ble_security_(this->ble_peer_bda_);
  } else if (event == ESP_GATTS_DISCONNECT_EVT) {
    this->ble_peer_connected_ = false;
    std::memset(this->ble_peer_bda_, 0, sizeof(esp_bd_addr_t));
  }
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_gatts_event_handler(event, gatts_if, param);
#else
  (void) gatts_if;
#endif
}
#endif

float CO2Monitor0601::battery_percent_from_voltage_(float voltage) {
  // Approximate single-cell Li-ion/LiPo state of charge under a light load.
  // The curve is intentionally conservative near the empty end. Voltage-based
  // SOC is an estimate; chemistry, load and temperature all shift the curve.
  struct Point { float v; float pct; };
  static const Point curve[] = {
      {3.30f, 0.0f}, {3.55f, 10.0f}, {3.65f, 20.0f}, {3.70f, 30.0f},
      {3.74f, 40.0f}, {3.79f, 50.0f}, {3.85f, 60.0f}, {3.92f, 70.0f},
      {4.00f, 80.0f}, {4.10f, 90.0f}, {4.20f, 100.0f},
  };

  if (voltage <= curve[0].v) return 0.0f;
  constexpr size_t n = sizeof(curve) / sizeof(curve[0]);
  if (voltage >= curve[n - 1].v) return 100.0f;
  for (size_t i = 1; i < n; i++) {
    if (voltage <= curve[i].v) {
      const float span = curve[i].v - curve[i - 1].v;
      const float t = (voltage - curve[i - 1].v) / span;
      return curve[i - 1].pct + t * (curve[i].pct - curve[i - 1].pct);
    }
  }
  return 0.0f;
}



void CO2Monitor0601::setup_battery_learning_() {
  if (global_preferences == nullptr) return;
  this->battery_.learning_pref =
      global_preferences->make_preference<BatteryLearningPersisted>(BATTERY_LEARNING_PREF_KEY, true);
  this->battery_.learning_pref_ready = true;

  BatteryLearningPersisted saved{};
  if (!this->battery_.learning_pref.load(&saved) || saved.version != BATTERY_LEARNING_VERSION) {
    this->publish_battery_learning_();
    return;
  }

  if (std::isfinite(saved.learned_full_runtime_h) && saved.learned_full_runtime_h >= 5.0f &&
      saved.learned_full_runtime_h <= 200.0f) {
    this->battery_.learned_full_runtime_h = saved.learned_full_runtime_h;
  }
  this->battery_.learned_cycles = saved.learned_cycles;
  if (saved.session_active && std::isfinite(saved.session_start_progress) &&
      std::isfinite(saved.session_last_progress)) {
    this->battery_.learning_session_active = true;
    this->battery_.learning_session_start_progress = saved.session_start_progress;
    this->battery_.learning_session_last_progress = saved.session_last_progress;
    this->battery_.learning_session_elapsed_ms = saved.session_elapsed_minutes * 60000UL;
    ESP_LOGI(TAG, "Battery learning: restored session %.1f -> %.1f %% after %.1f h",
             saved.session_start_progress, saved.session_last_progress,
             this->battery_.learning_session_elapsed_ms / 3600000.0f);
  }
  if (std::isfinite(this->battery_.learned_full_runtime_h)) {
    ESP_LOGI(TAG, "Battery learning: restored %.1f h full-runtime model from %u completed session(s)",
             this->battery_.learned_full_runtime_h,
             static_cast<unsigned>(this->battery_.learned_cycles));
  }
  this->publish_battery_learning_();
}

void CO2Monitor0601::prepare_for_ota() {
  // OTA blocks the normal application loop. Capture the time since the last
  // battery-learning tick before forcing the preference checkpoint so an OTA
  // update does not lose the tail of the current discharge session.
  if (this->battery_.learning_session_active && this->battery_.learning_last_tick_ms != 0) {
    const uint32_t now = millis();
    this->battery_.learning_session_elapsed_ms +=
        static_cast<uint32_t>(now - this->battery_.learning_last_tick_ms);
    this->battery_.learning_last_tick_ms = now;
  }

  ESP_LOGI(TAG, "OTA starting: flushing persistent runtime state");
  this->save_battery_learning_(true, "OTA");

#if UNNI_BLE_HISTORY_ENABLED
  if (!sensirion_history_flush())
    ESP_LOGW(TAG, "OTA starting: Sensirion history flash flush failed");
#endif
#if UNNI_BLE_ENABLED
  if (!sensirion_settings_flush())
    ESP_LOGW(TAG, "OTA starting: Sensirion Device Settings flush failed");
#endif
}

void CO2Monitor0601::publish_battery_learning_() {
  publish(this->out_.battery_learned_full_runtime, this->battery_.learned_full_runtime_h);
  publish(this->out_.battery_learning_cycles, static_cast<float>(this->battery_.learned_cycles));

  float progress_pct = 0.0f;
  if (this->battery_.learning_session_active &&
      std::isfinite(this->battery_.learning_session_start_progress) &&
      std::isfinite(this->battery_.learning_session_last_progress)) {
    const float drop = std::max(0.0f, this->battery_.learning_session_start_progress -
                                      this->battery_.learning_session_last_progress);
    const float time_factor = std::min(1.0f, this->battery_.learning_session_elapsed_ms /
                                                (2.0f * 3600000.0f));
    const float drop_factor = std::min(1.0f, drop / 8.0f);
    progress_pct = 100.0f * std::min(time_factor, drop_factor);
  }
  publish(this->out_.battery_learning_progress, progress_pct);
}

void CO2Monitor0601::save_battery_learning_(bool force, const char *reason) {
  if (!this->battery_.learning_pref_ready) return;
  const uint32_t now = millis();
  if (!force && this->battery_.learning_last_save_ms != 0 &&
      static_cast<uint32_t>(now - this->battery_.learning_last_save_ms) <
          this->battery_.learning_save_interval_ms) return;

  BatteryLearningPersisted saved{};
  saved.version = BATTERY_LEARNING_VERSION;
  saved.learned_cycles = this->battery_.learned_cycles;
  saved.learned_full_runtime_h = this->battery_.learned_full_runtime_h;
  saved.session_active = this->battery_.learning_session_active ? 1 : 0;
  saved.session_start_progress = this->battery_.learning_session_start_progress;
  saved.session_last_progress = this->battery_.learning_session_last_progress;
  saved.session_elapsed_minutes = this->battery_.learning_session_elapsed_ms / 60000UL;
  if (this->battery_.learning_pref.save(&saved)) {
    this->battery_.learning_last_save_ms = now;
    // Checkpoints are intentionally infrequent (30 min by default), so commit
    // each one to flash rather than leaving an overnight calibration only in
    // the deferred preference cache.
    if (global_preferences != nullptr) global_preferences->sync();

    float progress_pct = 0.0f;
    if (this->battery_.learning_session_active &&
        std::isfinite(this->battery_.learning_session_start_progress) &&
        std::isfinite(this->battery_.learning_session_last_progress)) {
      const float drop = std::max(0.0f, this->battery_.learning_session_start_progress -
                                        this->battery_.learning_session_last_progress);
      const float time_factor = std::min(1.0f, this->battery_.learning_session_elapsed_ms /
                                                  (2.0f * 3600000.0f));
      const float drop_factor = std::min(1.0f, drop / 8.0f);
      progress_pct = 100.0f * std::min(time_factor, drop_factor);
    }

    const char *checkpoint_kind = reason != nullptr ? reason : "periodic";
    ESP_LOGI(TAG,
             "Battery learning: %s checkpoint saved: SOC %.1f %%, elapsed %.2f h, progress %.0f %%, "
             "learned %.1f h, cycles %u",
             checkpoint_kind,
             this->battery_.learning_session_last_progress,
             this->battery_.learning_session_elapsed_ms / 3600000.0f,
             progress_pct,
             this->battery_.learned_full_runtime_h,
             static_cast<unsigned>(this->battery_.learned_cycles));
  } else {
    ESP_LOGW(TAG, "Battery learning: failed to save %s checkpoint",
             reason != nullptr ? reason : "periodic");
  }
}

void CO2Monitor0601::finalize_battery_learning_(bool completed_session) {
  if (!this->battery_.learning_session_active) return;
  const float drop = this->battery_.learning_session_start_progress -
                     this->battery_.learning_session_last_progress;
  const float elapsed_h = this->battery_.learning_session_elapsed_ms / 3600000.0f;

  if (completed_session && elapsed_h >= 2.0f && drop >= 8.0f) {
    const float observed_full_h = elapsed_h * 100.0f / drop;
    if (std::isfinite(observed_full_h) && observed_full_h >= 5.0f && observed_full_h <= 200.0f) {
      constexpr float LEARN_ALPHA = 0.25f;
      this->battery_.learned_full_runtime_h =
          std::isfinite(this->battery_.learned_full_runtime_h)
              ? (LEARN_ALPHA * observed_full_h + (1.0f - LEARN_ALPHA) *
                                               this->battery_.learned_full_runtime_h)
              : observed_full_h;
      if (this->battery_.learned_cycles < 65535U) this->battery_.learned_cycles++;
      ESP_LOGI(TAG, "Battery learning: session %.1f h / %.1f %% -> %.1f h full runtime; learned=%.1f h (%u)",
               elapsed_h, drop, observed_full_h, this->battery_.learned_full_runtime_h,
               static_cast<unsigned>(this->battery_.learned_cycles));
    }
  }

  this->battery_.learning_session_active = false;
  this->battery_.learning_session_elapsed_ms = 0;
  this->battery_.learning_session_start_progress = NAN;
  this->battery_.learning_session_last_progress = NAN;
  this->battery_.learning_last_tick_ms = 0;
  this->publish_battery_learning_();
  this->save_battery_learning_(true, "session-end");
}

void CO2Monitor0601::update_battery_learning_(float progress, uint32_t now) {
  if (!this->battery_.learning_session_active) {
    this->battery_.learning_session_active = true;
    this->battery_.learning_session_start_progress = progress;
    this->battery_.learning_session_last_progress = progress;
    this->battery_.learning_session_elapsed_ms = 0;
    this->battery_.learning_last_tick_ms = now;
    this->battery_.learning_last_save_ms = now;
    ESP_LOGI(TAG, "Battery learning: started session at %.1f %%", progress);
  } else {
    if (this->battery_.learning_last_tick_ms != 0)
      this->battery_.learning_session_elapsed_ms +=
          static_cast<uint32_t>(now - this->battery_.learning_last_tick_ms);
    this->battery_.learning_last_tick_ms = now;
    this->battery_.learning_session_last_progress = progress;
  }

  this->publish_battery_learning_();
  this->save_battery_learning_(false);
}

void CO2Monitor0601::reset_battery_estimator_(bool usb_mode, uint32_t now) {
  if (usb_mode && this->battery_.learning_session_active)
    this->finalize_battery_learning_(true);
  this->battery_.estimator_mode_usb = usb_mode;
  this->battery_.estimator_have_mode = true;
  this->battery_.estimator_have_anchor = false;
  this->battery_.estimator_mode_since_ms = now;

  // Learn each power-mode session independently. A source transition causes a
  // voltage step that must never be mistaken for charge/discharge progress.
  if (usb_mode) {
    this->battery_.charge_rate_valid = false;
    this->battery_.charge_rate_pct_h = NAN;
    publish(this->out_.battery_charge_time_estimate, NAN);
    publish(this->out_.battery_charge_rate, NAN);
    publish(this->out_.battery_runtime_estimate, NAN);
    publish(this->out_.battery_discharge_rate, NAN);
  } else {
    this->battery_.discharge_rate_valid = false;
    this->battery_.discharge_rate_pct_h = NAN;
    publish(this->out_.battery_runtime_estimate, NAN);
    publish(this->out_.battery_discharge_rate, NAN);
    publish(this->out_.battery_charge_time_estimate, NAN);
    publish(this->out_.battery_charge_rate, NAN);
  }
}

void CO2Monitor0601::update_battery_estimator_(float battery_voltage, bool usb_mode, uint32_t now) {
  if (!this->battery_.estimator_have_mode || this->battery_.estimator_mode_usb != usb_mode)
    this->reset_battery_estimator_(usb_mode, now);

  // Ignore the first two minutes after a source change. Terminal voltage
  // rebounds on discharge and rises immediately when the charger is attached.
  constexpr uint32_t SETTLE_MS = 2UL * 60UL * 1000UL;
  if (static_cast<uint32_t>(now - this->battery_.estimator_mode_since_ms) < SETTLE_MS) return;

  // On battery this is the normal voltage-derived SOC. With VBUS present it is
  // only a charge-progress proxy; Battery Level intentionally remains NaN.
  const float progress = battery_percent_from_voltage_(battery_voltage);
  if (!usb_mode) this->update_battery_learning_(progress, now);
  if (!this->battery_.estimator_have_anchor) {
    this->battery_.estimator_have_anchor = true;
    this->battery_.estimator_anchor_ms = now;
    this->battery_.estimator_anchor_progress = progress;
    return;
  }

  const uint32_t elapsed_ms = static_cast<uint32_t>(now - this->battery_.estimator_anchor_ms);
  const float delta = progress - this->battery_.estimator_anchor_progress;
  constexpr uint32_t MIN_WINDOW_MS = 5UL * 60UL * 1000UL;
  constexpr uint32_t MAX_WINDOW_MS = 15UL * 60UL * 1000UL;
  constexpr float MIN_PROGRESS_DELTA = 0.20f;
  if (elapsed_ms < MIN_WINDOW_MS) return;
  if (std::fabs(delta) < MIN_PROGRESS_DELTA && elapsed_ms < MAX_WINDOW_MS) return;

  const float hours = static_cast<float>(elapsed_ms) / 3600000.0f;
  float observed_rate = usb_mode ? (delta / hours) : (-delta / hours);

  // Wrong-direction changes are usually voltage relaxation/noise. Move the
  // anchor forward but do not contaminate the learned rate.
  if (!std::isfinite(observed_rate) || observed_rate <= 0.05f || observed_rate > 250.0f) {
    this->battery_.estimator_anchor_ms = now;
    this->battery_.estimator_anchor_progress = progress;
    return;
  }

  constexpr float EMA_ALPHA = 0.35f;
  float &rate = usb_mode ? this->battery_.charge_rate_pct_h : this->battery_.discharge_rate_pct_h;
  bool &valid = usb_mode ? this->battery_.charge_rate_valid : this->battery_.discharge_rate_valid;
  rate = valid ? (EMA_ALPHA * observed_rate + (1.0f - EMA_ALPHA) * rate) : observed_rate;
  valid = true;

  this->battery_.estimator_anchor_ms = now;
  this->battery_.estimator_anchor_progress = progress;

  if (usb_mode) {
    publish(this->out_.battery_charge_rate, rate);
    const float remaining_pct = std::max(0.0f, 100.0f - progress);
    const float eta_h = remaining_pct / rate;
    publish(this->out_.battery_charge_time_estimate, eta_h);
    ESP_LOGD(TAG, "Battery charge estimate: proxy %.1f %% / %.2f %%/h -> %.2f h remaining",
             progress, rate, eta_h);
  } else {
    publish(this->out_.battery_discharge_rate, rate);
    const float recent_eta_h = progress / rate;

    float session_full_h = NAN;
    if (this->battery_.learning_session_active) {
      const float drop = this->battery_.learning_session_start_progress - progress;
      const float elapsed_h = this->battery_.learning_session_elapsed_ms / 3600000.0f;
      if (elapsed_h >= 2.0f && drop >= 8.0f) session_full_h = elapsed_h * 100.0f / drop;
    }

    float model_full_h = this->battery_.learned_full_runtime_h;
    if (std::isfinite(session_full_h) && session_full_h >= 5.0f && session_full_h <= 200.0f)
      model_full_h = std::isfinite(model_full_h) ? (0.7f * model_full_h + 0.3f * session_full_h)
                                                : session_full_h;

    float eta_h = recent_eta_h;
    if (std::isfinite(model_full_h)) {
      const float model_eta_h = model_full_h * progress / 100.0f;
      eta_h = 0.7f * model_eta_h + 0.3f * recent_eta_h;
    }
    publish(this->out_.battery_runtime_estimate, eta_h);
    ESP_LOGD(TAG, "Battery runtime estimate: %.1f %% / %.2f %%/h -> %.2f h remaining%s",
             progress, rate, eta_h, std::isfinite(model_full_h) ? " (learned blend)" : "");
  }
}


#if UNNI_BLE_ENABLED
void BlePairingModeSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_ble_pairing_mode(state);
  else
    this->publish_state(state);
}

void CO2Monitor0601::begin_ble_security_(esp_bd_addr_t remote_bda) {
  const esp_err_t err = esp_ble_set_encryption(remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
  if (err != ESP_OK)
    ESP_LOGW(TAG, "BLE pairing: esp_ble_set_encryption failed: %s", esp_err_to_name(err));
  else
    ESP_LOGW(TAG, "BLE pairing: requested authenticated encryption");
}

void CO2Monitor0601::set_ble_pairing_mode(bool enabled) {
  this->ble_pairing_mode_ = enabled;
  this->ble_pairing_started_ms_ = enabled ? millis() : 0;
  if (this->ble_pairing_switch_ != nullptr) this->ble_pairing_switch_->publish_state(enabled);
  if (enabled) {
    ESP_LOGW(TAG, "BLE Pairing Mode: ON (%lu ms authorization window; connect with MyAmbience now)",
             static_cast<unsigned long>(this->ble_pairing_window_ms_));
    // MyAmbience may already have the single supported GATT connection open
    // when the HA switch is enabled. Initiate MITM encryption immediately in
    // that case instead of requiring a disconnect/reconnect.
    if (this->ble_peer_connected_) this->begin_ble_security_(this->ble_peer_bda_);
  } else
    ESP_LOGW(TAG, "BLE Pairing Mode: OFF");
}

void CO2Monitor0601::process_ble_pairing_window_() {
  if (!this->ble_pairing_mode_) return;
  if (static_cast<uint32_t>(millis() - this->ble_pairing_started_ms_) < this->ble_pairing_window_ms_) return;
  ESP_LOGW(TAG, "BLE Pairing Mode expired");
  this->set_ble_pairing_mode(false);
}

#endif

void WifiHaSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_wifi_ha_enabled(state);
  else
    this->publish_state(state);
}

void CO2Monitor0601::open_wifi_recovery_window_(uint32_t now, const char *reason) {
  if (this->wifi_ha_enabled_) return;
  this->wifi_recovery_until_ms_ = now + this->wifi_recovery_window_ms_;
  this->wifi_recovery_logged_ = false;
#ifdef USE_WIFI
  if (wifi::global_wifi_component != nullptr && wifi::global_wifi_component->is_disabled())
    wifi::global_wifi_component->enable();
#endif
  ESP_LOGW(TAG, "WiFi/HA recovery window opened for %lu ms (%s); turn the switch ON to keep WiFi enabled",
           static_cast<unsigned long>(this->wifi_recovery_window_ms_), reason);
}

#if UNNI_BLE_ENABLED
void CO2Monitor0601::sync_wifi_ha_from_sensirion_settings_() {
  const bool enabled = !sensirion_settings_ha_disabled();
  if (enabled == this->wifi_ha_enabled_) return;
  ESP_LOGW(TAG, "MyAmbience 0x81FE changed: WiFi Home Assistant -> %s",
           enabled ? "ON" : "OFF");
  // Apply without writing the same setting back; the GATT write already
  // scheduled persistence in normal loop context.
  this->wifi_ha_enabled_ = enabled;
  if (this->wifi_ha_switch_ != nullptr) this->wifi_ha_switch_->publish_state(enabled);
  if (enabled) {
    this->wifi_disable_pending_ = false;
    this->wifi_recovery_until_ms_ = 0;
#ifdef USE_WIFI
    if (wifi::global_wifi_component != nullptr && wifi::global_wifi_component->is_disabled())
      wifi::global_wifi_component->enable();
#endif
    this->publish_cached_ha_now_();
  } else {
    this->wifi_disable_pending_ = true;
    this->wifi_disable_requested_ms_ = millis();
    this->wifi_recovery_until_ms_ = 0;
    ESP_LOGW(TAG, "MyAmbience requested HA/WiFi disable; WiFi will stop in 1500 ms");
  }
}
#endif

void CO2Monitor0601::set_wifi_ha_enabled(bool enabled) {
#if UNNI_BLE_ENABLED
  // Keep the MyAmbience 0x81FE control synchronized with the HA switch.
  // 0x81FE uses the inverse sense: true means HA/WiFi disabled.
  sensirion_settings_set_ha_disabled(!enabled);
#endif
  this->wifi_ha_enabled_ = enabled;
  if (this->wifi_ha_switch_ != nullptr) this->wifi_ha_switch_->publish_state(enabled);
  if (enabled) {
    this->wifi_disable_pending_ = false;
    this->wifi_recovery_until_ms_ = 0;
#ifdef USE_WIFI
    if (wifi::global_wifi_component != nullptr && wifi::global_wifi_component->is_disabled())
      wifi::global_wifi_component->enable();
#endif
    ESP_LOGI(TAG, "WiFi Home Assistant: ON");
    this->publish_cached_ha_now_();
    return;
  }

  // Give the API response and switch state a moment to reach Home Assistant
  // before the interface disappears.
  this->wifi_disable_pending_ = true;
  this->wifi_disable_requested_ms_ = millis();
  this->wifi_recovery_until_ms_ = 0;
  ESP_LOGW(TAG, "WiFi Home Assistant: OFF requested; WiFi will stop in 1500 ms");
  ESP_LOGW(TAG, "Recovery: reconnect USB power (or reboot with USB attached) for a temporary HA window");
}

void CO2Monitor0601::process_wifi_ha_control_() {
#ifdef USE_WIFI
  if (wifi::global_wifi_component == nullptr) return;
  if (this->usb_power_.initialized && !this->usb_power_.have_state) return;
  const uint32_t now = millis();
  if (this->wifi_ha_enabled_) {
    if (wifi::global_wifi_component->is_disabled()) wifi::global_wifi_component->enable();
    return;
  }

  if (this->wifi_disable_pending_) {
    if (static_cast<uint32_t>(now - this->wifi_disable_requested_ms_) < 1500U) return;
    this->wifi_disable_pending_ = false;
    ESP_LOGI(TAG, "Disabling WiFi and Home Assistant connectivity");
    wifi::global_wifi_component->disable();
    return;
  }

  if (this->wifi_recovery_until_ms_ != 0 && static_cast<int32_t>(this->wifi_recovery_until_ms_ - now) > 0) {
    if (wifi::global_wifi_component->is_disabled()) wifi::global_wifi_component->enable();
    if (!this->wifi_recovery_logged_) {
      this->wifi_recovery_logged_ = true;
      ESP_LOGI(TAG, "WiFi Home Assistant temporarily enabled for USB recovery");
    }
    return;
  }

  if (this->wifi_recovery_until_ms_ != 0) {
    this->wifi_recovery_until_ms_ = 0;
    ESP_LOGI(TAG, "WiFi/HA recovery window expired; disabling WiFi again");
  }
  if (!wifi::global_wifi_component->is_disabled()) wifi::global_wifi_component->disable();
#endif
}

void EnergySaveModeSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_energy_save_mode(state);
  else
    this->publish_state(state);
}

void CO2Monitor0601::set_energy_save_mode(bool enabled) {
  if (this->energy_save_mode_ == enabled) {
    if (this->energy_save_switch_ != nullptr) this->energy_save_switch_->publish_state(enabled);
    return;
  }

  this->energy_save_mode_ = enabled;
  if (this->energy_save_switch_ != nullptr) this->energy_save_switch_->publish_state(enabled);

  if (!enabled) {
    this->energy_save_grace_pending_ = false;
    this->energy_save_policy_active_ = false;
    ESP_LOGI(TAG, "Energy Save Mode: OFF (automatic USB/battery policy)");
    this->apply_power_policy_(true);
    return;
  }

  if (this->usb_powered_() && this->energy_save_grace_ms_ > 0) {
    // Keep the normal USB PM locks briefly so HA receives the switch update and
    // final log messages before native USB Serial/JTAG may disappear in sleep.
    this->energy_save_policy_active_ = false;
    this->energy_save_grace_pending_ = true;
    this->energy_save_grace_started_ms_ = millis();
    ESP_LOGI(TAG, "Energy Save Mode: ON; battery policy starts in %lu ms",
             static_cast<unsigned long>(this->energy_save_grace_ms_));
    ESP_LOGW(TAG, "Native USB Serial/JTAG may disconnect once Light-sleep becomes active");
    this->apply_power_policy_(true);
    return;
  }

  this->energy_save_grace_pending_ = false;
  this->energy_save_policy_active_ = true;
  ESP_LOGI(TAG, "Energy Save Mode: ON (battery policy forced)");
  ESP_LOGW(TAG, "Native USB Serial/JTAG may disconnect while Light-sleep is active");
  this->apply_power_policy_(true);
}

void CO2Monitor0601::process_energy_save_grace_() {
  if (!this->energy_save_grace_pending_) return;
  if (!this->energy_save_mode_) {
    this->energy_save_grace_pending_ = false;
    return;
  }
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - this->energy_save_grace_started_ms_) < this->energy_save_grace_ms_) return;

  this->energy_save_grace_pending_ = false;
  this->energy_save_policy_active_ = true;
  ESP_LOGI(TAG, "Energy Save Mode grace period complete; enabling battery policy");
  ESP_LOGI(TAG, "WiFi modem sleep remains enabled; native USB Serial/JTAG may disconnect");
  this->apply_power_policy_(true);
}

void CO2Monitor0601::apply_power_policy_(bool force) {
  // Do not make policy decisions from an undebounced VBUS input. The configured
  // default is applied as soon as process_usb_power_() has established the
  // first physical USB state.
  if (this->usb_power_.initialized && !this->usb_power_.have_state) return;

  const bool external = this->external_powered_();
  if (!force && this->power_policy_have_state_ && this->power_policy_external_power_ == external) return;

  this->power_policy_have_state_ = true;
  this->power_policy_external_power_ = external;
  power_save::set_external_power(external);

  // A policy transition starts a fresh CO2 observation window. USB keeps the
  // passive sniffer continuously active; battery mode may gate it again after
  // the native CO2 measurement window has been satisfied.
  if (this->io_initialized_)
    this->reset_co2_capture_gate_();

#if UNNI_BLE_ENABLED
  const uint32_t adv_ms = external ? this->ble_usb_advertising_interval_ms_
                                   : this->ble_battery_advertising_interval_ms_;
  sensirion_ble_set_advertising_interval(adv_ms);
  ESP_LOGI(TAG, "BLE advertising interval: %lu ms (%s policy)",
           static_cast<unsigned long>(adv_ms), external ? "USB" : "battery");
#endif

  const uint32_t now = millis();
  if (external) {
    // Returning to normal USB policy should immediately expose the freshest
    // cached values instead of waiting for the next sensor cycle.
    this->publish_cached_ha_now_();
  } else {
    // Battery policy publishes Home Assistant values only at its configured
    // interval, regardless of whether VBUS is physically present.
    this->ha_.last_publish_ms = now;
  }

  ESP_LOGI(TAG, "Power policy: %s%s", external ? "USB" : "battery",
           this->energy_save_policy_active_ ? " (Energy Save Mode override)" : "");
}

bool CO2Monitor0601::setup_usb_power_() {
  gpio_config_t cfg{};
  cfg.pin_bit_mask = 1ULL << this->usb_power_.pin;
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  const esp_err_t err = gpio_config(&cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB power GPIO%u setup failed: %d", this->usb_power_.pin, err);
    return false;
  }

  this->usb_power_.initialized = true;
  this->usb_power_.candidate = gpio_get_level(static_cast<gpio_num_t>(this->usb_power_.pin)) != 0;
  this->usb_power_.candidate_since_ms = millis();
  ESP_LOGI(TAG, "USB/VBUS detection enabled on GPIO%u", this->usb_power_.pin);
  return true;
}

void CO2Monitor0601::process_usb_power_() {
  if (!this->usb_power_.initialized) return;

  const uint32_t now = millis();
  constexpr uint32_t STABLE_POLL_MS = 200;
  constexpr uint32_t DEBOUNCE_POLL_MS = 10;
  const bool candidate_active = !this->usb_power_.have_state ||
                                this->usb_power_.candidate != this->usb_power_.state;
  const uint32_t poll_interval = candidate_active ? DEBOUNCE_POLL_MS : STABLE_POLL_MS;
  if (this->usb_power_.last_poll_ms != 0 &&
      static_cast<uint32_t>(now - this->usb_power_.last_poll_ms) < poll_interval)
    return;
  this->usb_power_.last_poll_ms = now;
  const bool raw = gpio_get_level(static_cast<gpio_num_t>(this->usb_power_.pin)) != 0;
  if (raw != this->usb_power_.candidate) {
    this->usb_power_.candidate = raw;
    this->usb_power_.candidate_since_ms = now;
    return;
  }

  // USB insertion/removal is slow compared with the MCU. Debounce the divider
  // input so a transition cannot briefly invalidate/re-enable battery SOC.
  constexpr uint32_t DEBOUNCE_MS = 100;
  if (static_cast<uint32_t>(now - this->usb_power_.candidate_since_ms) < DEBOUNCE_MS) return;
  if (this->usb_power_.have_state && this->usb_power_.state == raw) return;

  this->usb_power_.state = raw;
  this->usb_power_.have_state = true;
  publish(this->out_.usb_power, raw);
  ESP_LOGI(TAG, "USB Power: %s", raw ? "ON" : "OFF");
  if (raw && !this->wifi_ha_enabled_) this->open_wifi_recovery_window_(now, "USB power detected");

  // Keep the physical USB entity truthful, but let Energy Save Mode override
  // the runtime policy so USB power meters can measure the same behavior used
  // on battery.
  this->apply_power_policy_();

  // Cell voltage is not a useful open-circuit SOC estimate while USB is
  // present and the battery node may be driven by the charger. Mark Battery
  // Level unavailable until the device is back on battery power.
  if (raw && this->out_.battery_level)
    this->out_.battery_level->publish_state(NAN);

  // A power-source transition causes an immediate battery-node voltage step.
  // Restart ETA learning so that step cannot be interpreted as charge/discharge.
  this->reset_battery_estimator_(raw, now);

  // Measure immediately after a power-source transition rather than waiting
  // for the regular battery interval.
  this->battery_.last_measure_ms = 0;
}

bool CO2Monitor0601::setup_battery_adc_() {
  // ESP32-C3 GPIO0..GPIO4 map directly to ADC1 channels 0..4. Keeping the
  // battery input on ADC1 avoids the C3 ADC2/Wi-Fi limitations.
  if (this->battery_.pin > 4) {
    ESP_LOGE(TAG, "Battery GPIO%u is not an ESP32-C3 ADC1 pin", this->battery_.pin);
    return false;
  }
  this->battery_.unit = ADC_UNIT_1;
  this->battery_.channel = static_cast<adc_channel_t>(this->battery_.pin);

  adc_oneshot_unit_init_cfg_t unit_cfg{};
  unit_cfg.unit_id = this->battery_.unit;
  unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
  esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &this->battery_.adc_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Battery ADC unit setup failed: %d", err);
    return false;
  }

  adc_oneshot_chan_cfg_t chan_cfg{};
  chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
  chan_cfg.atten = ADC_ATTEN_DB_12;
  err = adc_oneshot_config_channel(this->battery_.adc_handle, this->battery_.channel, &chan_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Battery ADC channel setup failed: %d", err);
    return false;
  }

  adc_cali_curve_fitting_config_t cali_cfg{};
  cali_cfg.unit_id = this->battery_.unit;
  cali_cfg.chan = this->battery_.channel;
  cali_cfg.atten = ADC_ATTEN_DB_12;
  cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
  err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &this->battery_.cali_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Battery ADC calibration unavailable (%d); battery entities disabled", err);
    return false;
  }

  this->battery_.initialized = true;
  ESP_LOGI(TAG, "Battery ADC enabled on GPIO%u / ADC1_CH%u, divider %.3fx",
           this->battery_.pin, static_cast<unsigned>(this->battery_.channel),
           this->battery_.divider_ratio);
  return true;
}

void CO2Monitor0601::process_battery_() {
  if (!this->battery_.initialized) return;
  // Wait for the debounced VBUS state before interpreting the battery node.
  // This avoids briefly reporting a charger-held ~4.2 V node as 100% SOC at boot.
  if (this->usb_power_.initialized && !this->usb_power_.have_state) return;
  const uint32_t now = millis();
  if (this->battery_.last_measure_ms != 0 &&
      static_cast<uint32_t>(now - this->battery_.last_measure_ms) < this->battery_.interval_ms)
    return;

  // Never insert an ADC polling conversion into the timing-critical capture
  // window. Wait until Light-sleep is permitted again.
  if (power_save::enabled() && power_save::awake_window_active()) return;

  // Average several calibrated samples. The external 100 nF capacitor provides
  // a low-impedance source for the ADC despite the 1 MΩ / 1 MΩ divider.
  constexpr int SAMPLE_COUNT = 8;
  int64_t sum_mv = 0;
  int valid = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int raw = 0;
    if (adc_oneshot_read(this->battery_.adc_handle, this->battery_.channel, &raw) != ESP_OK)
      continue;
    int mv = 0;
    if (adc_cali_raw_to_voltage(this->battery_.cali_handle, raw, &mv) != ESP_OK)
      continue;
    sum_mv += mv;
    valid++;
  }
  this->battery_.last_measure_ms = now;
  if (valid == 0) {
    ESP_LOGW(TAG, "Battery ADC read failed");
    return;
  }

  const float adc_voltage = static_cast<float>(sum_mv) / static_cast<float>(valid) / 1000.0f;
  const float battery_voltage = adc_voltage * this->battery_.divider_ratio;
  publish(this->out_.battery_voltage, battery_voltage);

  if (this->usb_power_.have_state && this->usb_power_.state) {
    // With VBUS present this is the charger/BAT-node voltage and must not be
    // interpreted as an open-circuit battery state of charge.
    if (this->out_.battery_level) this->out_.battery_level->publish_state(NAN);
    ESP_LOGD(TAG, "Battery node: %.3f V (USB power present; SOC unavailable)", battery_voltage);
    this->update_battery_estimator_(battery_voltage, true, now);
  } else {
    const float battery_level = battery_percent_from_voltage_(battery_voltage);
    publish(this->out_.battery_level, battery_level);
    ESP_LOGD(TAG, "Battery: %.3f V -> %.0f %%", battery_voltage, battery_level);
    this->update_battery_estimator_(battery_voltage, false, now);
  }
}

bool CO2Monitor0601::initialize_sniffer_io_() {
  if (this->io_initialized_) return true;
  if (this->io_initialization_attempted_) return false;
  this->io_initialization_attempted_ = true;

  if (!i2c_sniffer::setup(this->co2_sda_pin_, this->co2_scl_pin_, this->i2c_rmt_scl_assist_)) {
    ESP_LOGE(TAG, "I2C sniffer GPIO/ISR setup failed");
    i2c_sniffer::shutdown();
    this->mark_failed();
    return false;
  }
  const bool setup_rtrh_gpio = this->rtrh_enabled_ || this->rtrh_gpio_setup_ ||
                                this->rtrh_edge_capture_ || this->rtrh_decode_only_;
  if (setup_rtrh_gpio) {
    const bool enable_rtrh_edge_isr = this->rtrh_enabled_ || this->rtrh_edge_capture_ ||
                                      this->rtrh_decode_only_;
    if (!enable_rtrh_edge_isr) {
      ESP_LOGE(TAG, "Known-good RT/RH restore test requires edge capture enabled");
      i2c_sniffer::shutdown();
      this->mark_failed();
      return false;
    }
    // A/B: use the exact pre-regression RT/RH setup path. In particular,
    // GPIO_INTR_ANYEDGE is configured before the initial GPIO snapshots, just
    // as in the 16:05 build that produced valid temperature/humidity samples.
    if (!rtrh_decoder::setup(this->rt_pin_, this->rh_pin_)) {
      ESP_LOGE(TAG, "RT/RH decoder GPIO/ISR setup failed");
      rtrh_decoder::shutdown();
      i2c_sniffer::shutdown();
      this->mark_failed();
      return false;
    }
    ESP_LOGW(TAG, "RT/RH A/B: exact known-good decoder and GPIO/ISR setup restored");
  } else {
    ESP_LOGW(TAG, "Capture A/B: RT/RH GPIO setup and edge ISR disabled; CO2 I2C sniffer only");
  }

  this->io_initialized_ = true;
  if (setup_rtrh_gpio) {
    if (!power_save::setup(this->light_sleep_enabled_, this->light_sleep_max_awake_ms_,
                           this->rt_pin_, this->rh_pin_,
                           this->co2_sda_pin_, this->co2_scl_pin_)) {
      ESP_LOGW(TAG, "Requested auto Light-sleep could not be enabled; continuing normally");
    } else if (power_save::enabled()) {
      // If VBUS was already debounced before delayed sniffer initialization,
      // immediately apply the matching USB/battery power policy.
      this->apply_power_policy_(true);
      i2c_sniffer::set_capture_enabled(true);
    }
  }
  ESP_LOGI(TAG, "Sniffer GPIO/ISR initialization enabled after %lu ms",
           static_cast<unsigned long>(millis() - this->boot_ms_));
  return true;
}

void CO2Monitor0601::setup() {
  ESP_LOGI(TAG, "ESPHome entities registered normally; API/Wi-Fi presence is controlled by YAML");
#if defined(USE_SENSOR) && defined(USE_BINARY_SENSOR) && defined(USE_SWITCH)
  ESP_LOGI(TAG, "ESPHome entity registry: sensors=%u binary_sensors=%u switches=%u",
           static_cast<unsigned>(App.get_sensors().size()),
           static_cast<unsigned>(App.get_binary_sensors().size()),
           static_cast<unsigned>(App.get_switches().size()));
#endif
#if UNNI_RUNTIME_DIAGNOSTICS
  ESP_LOGW(TAG, "Runtime diagnostics enabled: main-loop timing + heap telemetry; ISR paths unchanged");
#endif
  if (this->energy_save_switch_ != nullptr) this->energy_save_switch_->publish_state(this->energy_save_mode_);
  if (this->wifi_ha_switch_ != nullptr) {
#if UNNI_BLE_ENABLED
    // The persistent Sensirion 0x81FE setting is the source of truth when BLE
    // is present, so MyAmbience can always restore WiFi/HA after a reboot.
    this->wifi_ha_enabled_ = true;
#else
    const auto restored = this->wifi_ha_switch_->get_initial_state_with_restore_mode();
    this->wifi_ha_enabled_ = restored.value_or(true);
#endif
    this->wifi_ha_switch_->publish_state(this->wifi_ha_enabled_);
  }
#if UNNI_BLE_ENABLED
  if (this->ble_pairing_switch_ != nullptr) this->ble_pairing_switch_->publish_state(false);
#endif

  if (this->sniffer_enabled_) {
    // Claim the shared GPIO ISR service before Wi-Fi/BLE setup reaches steady
    // state, but without touching any sniffer signal pin. ESP_INTR_FLAG_IRAM
    // keeps the dispatcher and our IRAM_ATTR pin handlers callable while flash
    // cache is disabled. This preserves the configured GPIO isolation delay.
    const esp_err_t gpio_isr_err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (gpio_isr_err == ESP_OK) {
      ESP_LOGI(TAG, "GPIO ISR service installed IRAM-safe");
    } else if (gpio_isr_err == ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG, "GPIO ISR service was already installed; IRAM allocation flag cannot be verified");
    } else {
      ESP_LOGE(TAG, "gpio_install_isr_service(IRAM) failed: %d", gpio_isr_err);
    }
  } else {
    ESP_LOGW(TAG, "Sniffer A/B: GPIO ISR service and CO2/RT/RH capture disabled");
  }

#if UNNI_BLE_ENABLED
  sensirion_ble_setup();

  // GATT topology must be built at runtime, not from a codegen setter.
  // ESPHome creates the BLEServer object before all of its generated setup
  // statements (notably set_parent()) have necessarily been emitted. Calling
  // BLEServer::create_service() from codegen can therefore dereference an
  // unset parent. By setup() time all generated wiring is complete.
  if (this->gatt_server_ != nullptr) {
    auto *info = this->gatt_server_->get_service(esp32_ble::ESPBTUUID::from_uint16(0x180A));
    if (info != nullptr) {
#if UNNI_SHT43_IDENTITY_PROBE
      if (auto *manufacturer = info->get_characteristic(0x2A29))
        manufacturer->set_value(std::string("Sensirion"));
      if (auto *model = info->get_characteristic(0x2A24))
        model->set_value(std::string("SHT43 DB"));
      if (auto *firmware = info->get_characteristic(0x2A26))
        firmware->set_value(std::string("1.0.0"));
#else
      if (auto *manufacturer = info->get_characteristic(0x2A29))
        manufacturer->set_value(std::string("Gadget"));
      if (auto *model = info->get_characteristic(0x2A24))
        model->set_value(this->ble_device_name_);
      if (auto *firmware = info->get_characteristic(0x2A26))
        firmware->set_value(std::string("1.0.1"));
#endif
    }
#if UNNI_BLE_HISTORY_ENABLED
    sensirion_history_configure_gatt(this->gatt_server_);
#endif
#if UNNI_SHT43_IDENTITY_PROBE
    sensirion_sht43_probe_configure_gatt(this->gatt_server_);
    ESP_LOGW(TAG, "SHT43 GATT probe: serial + T/RH + secure Device Settings 0x8100 enabled");
    ESP_LOGI(TAG, "Heap before BLE enable: free=%u B, largest_8bit=%u B",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
#endif
    sensirion_settings_configure_gatt(this->gatt_server_, this->ble_device_name_);
    // 0x81FE (shown by MyAmbience as IsLogEnabled) doubles as our HA/WiFi
    // disable switch. This gives BLE a recovery/control path while WiFi is off.
    this->wifi_ha_enabled_ = !sensirion_settings_ha_disabled();
    if (this->wifi_ha_switch_ != nullptr)
      this->wifi_ha_switch_->publish_state(this->wifi_ha_enabled_);
    if (!this->wifi_ha_enabled_) {
      this->wifi_disable_pending_ = true;
      this->wifi_disable_requested_ms_ = millis();
      ESP_LOGW(TAG, "MyAmbience HA/WiFi disable restored; WiFi will stop after startup grace");
    }
  } else {
    ESP_LOGE(TAG, "BLE enabled but no GATT server instance is available");
  }
#endif
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_setup();
#endif

  this->setup_usb_power_();
  this->setup_battery_adc_();
  this->setup_battery_learning_();
#if RTRH_DEBUG_CAPTURE
  if (!this->debug_udp_host_.empty() && this->debug_udp_port_ != 0) {
    if (!debug_udp::setup(this->debug_udp_host_.c_str(), this->debug_udp_port_))
      ESP_LOGW(TAG, "UDP debug export configuration failed; continuing without network capture export");
  }
#endif
  this->boot_ms_ = millis();
  if (this->sniffer_enabled_) {
    if (this->start_delay_ms_ == 0) {
      if (!this->initialize_sniffer_io_()) return;
    } else {
      ESP_LOGI(TAG, "Sniffer GPIO isolation active for first %lu ms; signal pins untouched",
               static_cast<unsigned long>(this->start_delay_ms_));
    }

#if RTRH_DEBUG_CAPTURE
    i2c_sniffer::register_debug_handler();
    if (this->rtrh_enabled_ || this->rtrh_decode_only_ || this->rtrh_edge_capture_)
      rtrh_decoder::register_debug_handlers();
    if (debug_udp::enabled()) {
      ESP_LOGD(TAG, this->rtrh_enabled_ ? "Raw debug: UDP I2C + RT/RH capture export enabled"
                                       : "Raw debug: UDP I2C capture export enabled");
    } else {
#if defined(USE_WEB_SERVER_BASE)
      ESP_LOGD(TAG, this->rtrh_enabled_ ? "Raw debug: HTTP /capture, /rt_rh_capture.csv, /rt_rh_timing.csv"
                                       : "Raw debug: HTTP /capture (RT/RH capture disabled)");
#else
      ESP_LOGD(TAG, "Raw capture instrumentation enabled; no network export configured");
#endif
    }
#else
    if (this->rtrh_enabled_)
      ESP_LOGD(TAG, "RT/RH time-phase decoder active; debug capture disabled");
    else if (this->rtrh_decode_only_)
      ESP_LOGD(TAG, "RT/RH decoder A/B active; publication disabled; debug capture instrumentation enabled");
    else
      ESP_LOGD(TAG, "CO2-only capture A/B active; RT/RH decoder disabled");
#endif

    publish(this->out_.crc_errors, 0.0f);
    publish(this->out_.frame_errors, 0.0f);
    ESP_LOGI(TAG, "Passive CO2 + RT/RH sniffer ready");
  } else {
    ESP_LOGW(TAG, "Sniffer A/B active: no signal GPIO setup, no capture ISR, no RT/RH or CO2 decoding");
  }
}

void CO2Monitor0601::publish_cached_ha_now_() {
  bool published = false;
  if (this->ha_.have_co2 && this->out_.co2) {
    this->out_.co2->publish_state(this->ha_.co2);
    this->ha_.initial_co2_published = true;
    published = true;
  }
  if (this->ha_.have_temperature && this->out_.temperature) {
    this->out_.temperature->publish_state(this->ha_.temperature);
    this->ha_.initial_temperature_published = true;
    published = true;
  }
  if (this->ha_.have_air_temperature && this->out_.air_temperature) {
    this->out_.air_temperature->publish_state(this->ha_.air_temperature);
    this->ha_.initial_air_temperature_published = true;
    published = true;
  }
  if (this->ha_.have_display_temperature && this->out_.display_temperature) {
    this->out_.display_temperature->publish_state(this->ha_.display_temperature);
    this->ha_.initial_display_temperature_published = true;
    published = true;
  }
  if (this->ha_.have_humidity && this->out_.humidity) {
    this->out_.humidity->publish_state(this->ha_.humidity);
    this->ha_.initial_humidity_published = true;
    published = true;
  }
  if (this->ha_.have_display_humidity && this->out_.display_humidity) {
    this->out_.display_humidity->publish_state(this->ha_.display_humidity);
    this->ha_.initial_display_humidity_published = true;
    published = true;
  }
  if (published) this->ha_.last_publish_ms = millis();
}

void CO2Monitor0601::maybe_publish_ha_() {
  if (!this->wifi_ha_enabled_) return;
  // On USB power every fresh measurement is published directly from its
  // decoder. The interval below is deliberately a battery-mode throttle.
  if (this->external_powered_()) return;

  const uint32_t now = millis();
  if (this->ha_.last_publish_ms &&
      static_cast<uint32_t>(now - this->ha_.last_publish_ms) < this->ha_.interval_ms)
    return;

  bool published = false;
  if (this->ha_.have_co2 && this->out_.co2 && this->ha_.initial_co2_published) {
    this->out_.co2->publish_state(this->ha_.co2);
    published = true;
  }
  if (this->ha_.have_temperature && this->out_.temperature &&
      this->ha_.initial_temperature_published) {
    this->out_.temperature->publish_state(this->ha_.temperature);
    published = true;
  }
  if (this->ha_.have_air_temperature && this->out_.air_temperature &&
      this->ha_.initial_air_temperature_published) {
    this->out_.air_temperature->publish_state(this->ha_.air_temperature);
    published = true;
  }
  if (this->ha_.have_display_temperature && this->out_.display_temperature &&
      this->ha_.initial_display_temperature_published) {
    this->out_.display_temperature->publish_state(this->ha_.display_temperature);
    published = true;
  }
  if (this->ha_.have_humidity && this->out_.humidity &&
      this->ha_.initial_humidity_published) {
    this->out_.humidity->publish_state(this->ha_.humidity);
    published = true;
  }
  if (this->ha_.have_display_humidity && this->out_.display_humidity &&
      this->ha_.initial_display_humidity_published) {
    this->out_.display_humidity->publish_state(this->ha_.display_humidity);
    published = true;
  }
  if (published) this->ha_.last_publish_ms = now;
}

float CO2Monitor0601::update_thermal_transient_(float temperature_c) {
  const uint32_t now_ms = millis();
  float rate_c_per_min = 0.0f;

  if (this->thermal_.have_previous) {
    const uint32_t dt_ms = now_ms - this->thermal_.previous_ms;
    if (dt_ms) {
      rate_c_per_min = std::fabs(temperature_c - this->thermal_.previous_temperature) *
                       60000.0f / static_cast<float>(dt_ms);
    }
  }

  if (!this->thermal_.active) {
    if (this->thermal_.have_previous &&
        rate_c_per_min >= this->thermal_.on_rate)
      this->thermal_.active = true;
  } else if (rate_c_per_min <= this->thermal_.off_rate) {
    this->thermal_.active = false;
  }

  this->thermal_.previous_temperature = temperature_c;
  this->thermal_.previous_ms = now_ms;
  this->thermal_.have_previous = true;
  return rate_c_per_min;
}

void CO2Monitor0601::process_rtrh_(bool publish_outputs) {
  rtrh_decoder::loop();

  rtrh_decoder::Measurement m;
  if (!rtrh_decoder::poll(m)) return;
  power_save::on_rtrh_complete();
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_note_rtrh_cycle();
#endif

  ESP_LOGI(TAG,
           "RT/RH measurement %lu quality: REF %.3f us / %.3f ms / %u, "
           "RT %.3f us / %.3f ms / %u, RH %.3f ms / state %.3f us (%u/%lu) -> %s",
           static_cast<unsigned long>(m.sequence), m.ref_period_us, m.ref_duration_ms,
           static_cast<unsigned>(m.ref_count), m.rt_phase_period_us, m.rt_duration_ms,
           static_cast<unsigned>(m.rt_phase_count), m.rh_duration_ms, m.rh_state_us,
           static_cast<unsigned>(m.rh_state_samples), static_cast<unsigned long>(m.rh_state_seen),
           m.valid ? "VALID" : "REJECT");

  ESP_LOGI(TAG,
           "RT/RH RH carrier %lu: %.3f us / %u cycles, carrier/REF=%.6f; "
           "edges RT +%u/-%u RH +%u/-%u",
           static_cast<unsigned long>(m.sequence), m.rh_carrier_period_us,
           static_cast<unsigned>(m.rh_carrier_count), m.rh_carrier_ref_ratio,
           static_cast<unsigned>(m.rh_rt_rise_edges), static_cast<unsigned>(m.rh_rt_fall_edges),
           static_cast<unsigned>(m.rh_rh_rise_edges), static_cast<unsigned>(m.rh_rh_fall_edges));

  ESP_LOGI(TAG,
           "RT/RH RH phase %lu: rise RH-RT=%+.3f us (%u), fall=%+.3f us (%u), "
           "mean=%+.3f us, rise/carrier=%+.6f",
           static_cast<unsigned long>(m.sequence), m.rh_phase_rise_us,
           static_cast<unsigned>(m.rh_phase_rise_samples), m.rh_phase_fall_us,
           static_cast<unsigned>(m.rh_phase_fall_samples), m.rh_phase_mean_us,
           m.rh_phase_rise_carrier_ratio);

  if (m.rh_state_samples > 0) {
    ESP_LOGI(TAG,
             "RT/RH RH intervals %lu: min=%u p25=%u median=%.1f p75=%u max=%u us; "
             "~220=%u ~440=%u other=%u (retained=%u, seen=%lu)",
             static_cast<unsigned long>(m.sequence),
             static_cast<unsigned>(m.rh_state_min_us),
             static_cast<unsigned>(m.rh_state_p25_us), m.rh_state_us,
             static_cast<unsigned>(m.rh_state_p75_us),
             static_cast<unsigned>(m.rh_state_max_us),
             static_cast<unsigned>(m.rh_state_near_220),
             static_cast<unsigned>(m.rh_state_near_440),
             static_cast<unsigned>(m.rh_state_other),
             static_cast<unsigned>(m.rh_state_samples),
             static_cast<unsigned long>(m.rh_state_seen));
  }

  if (!m.valid) {
    if (publish_outputs) {
      publish(this->out_.quality, m.quality_percent);
      publish(this->out_.ref_period, m.ref_period_us);
      publish_positive(this->out_.rt_period, m.rt_period_us);
      publish_positive(this->out_.rh_state_period, m.rh_state_us);
      publish_finite(this->out_.rt_ratio, m.rt_ratio);
      publish_finite(this->out_.rh_ratio, m.rh_ratio);
      publish(this->out_.temperature_extrapolation, m.temperature_extrapolation);
      publish(this->out_.humidity_extrapolation, true);
      publish(this->out_.calibration_extrapolation, true);

      // A rejected capture may retain temperature only when the reject is
      // strictly RH-local and RT/REF remains continuous with the last accepted
      // measurement. Full-capture failures such as RH_DURATION must not feed
      // Temperature/Air Temperature/Display Temperature. This prevents wake
      // capture glitches from producing large false temperature jumps.
      const bool rh_local_reject =
          m.reject_reason == rtrh_decoder::RejectReason::RH_TOO_FEW_SAMPLES ||
          m.reject_reason == rtrh_decoder::RejectReason::RH_STATE_PERIOD ||
          m.reject_reason == rtrh_decoder::RejectReason::RH_RATIO_IMPLAUSIBLE ||
          m.reject_reason == rtrh_decoder::RejectReason::RH_CARRIER_COUNT ||
          m.reject_reason == rtrh_decoder::RejectReason::RH_CARRIER_PERIOD;
      constexpr float MAX_REJECT_RT_RATIO_STEP = 0.10f;
      const bool rt_ratio_continuous =
          this->rt_validity_.have_last_accepted_ratio && std::isfinite(m.rt_ratio) &&
          std::fabs(m.rt_ratio - this->rt_validity_.last_accepted_ratio) <=
              MAX_REJECT_RT_RATIO_STEP;
      const bool retain_rejected_temperature =
          rh_local_reject && m.temperature_valid && std::isfinite(m.temperature_c) &&
          rt_ratio_continuous;

      if (retain_rejected_temperature) {
        const float temperature_rate = this->update_thermal_transient_(m.temperature_c);
        (void) temperature_rate;
        m.thermal_transient = this->thermal_.active;
        publish(this->out_.thermal_transient, m.thermal_transient);

        this->ha_.temperature = m.temperature_c;
        if (std::isfinite(m.air_temperature_c)) {
          this->ha_.air_temperature = m.air_temperature_c;
          this->ha_.have_air_temperature = true;
        }
        if (std::isfinite(m.display_temperature_c)) {
          this->ha_.display_temperature = m.display_temperature_c;
          this->ha_.have_display_temperature = true;
        }
        this->ha_.have_temperature = true;
        if (this->external_powered_()) {
          if (this->out_.temperature) this->out_.temperature->publish_state(m.temperature_c);
          if (this->out_.air_temperature && std::isfinite(m.air_temperature_c))
            this->out_.air_temperature->publish_state(m.air_temperature_c);
          if (this->out_.display_temperature && std::isfinite(m.display_temperature_c))
            this->out_.display_temperature->publish_state(m.display_temperature_c);
          this->ha_.initial_temperature_published = this->out_.temperature != nullptr;
          this->ha_.initial_air_temperature_published =
              this->out_.air_temperature != nullptr && std::isfinite(m.air_temperature_c);
          this->ha_.initial_display_temperature_published =
              this->out_.display_temperature != nullptr && std::isfinite(m.display_temperature_c);
          this->ha_.last_publish_ms = millis();
        } else {
          if (!this->ha_.initial_temperature_published && this->out_.temperature) {
            this->out_.temperature->publish_state(m.temperature_c);
            this->ha_.initial_temperature_published = true;
            this->ha_.last_publish_ms = millis();
          }
          if (!this->ha_.initial_air_temperature_published && this->out_.air_temperature &&
              std::isfinite(m.air_temperature_c)) {
            this->out_.air_temperature->publish_state(m.air_temperature_c);
            this->ha_.initial_air_temperature_published = true;
            this->ha_.last_publish_ms = millis();
          }
          if (!this->ha_.initial_display_temperature_published && this->out_.display_temperature &&
              std::isfinite(m.display_temperature_c)) {
            this->out_.display_temperature->publish_state(m.display_temperature_c);
            this->ha_.initial_display_temperature_published = true;
            this->ha_.last_publish_ms = millis();
          }
        }

        this->rt_validity_.last_accepted_ratio = m.rt_ratio;
        this->rt_validity_.have_last_accepted_ratio = true;

        ESP_LOGI(TAG,
                 "RT/RH measurement %lu temperature retained despite RH-local reject: "
                 "%.3f / REF %.3f us = %.6f -> %.2f C",
                 static_cast<unsigned long>(m.sequence), m.rt_period_us, m.ref_period_us,
                 m.rt_ratio, m.temperature_c);
      } else if (publish_outputs && m.temperature_valid && std::isfinite(m.temperature_c)) {
        const float ratio_step = this->rt_validity_.have_last_accepted_ratio &&
                                         std::isfinite(m.rt_ratio)
                                     ? std::fabs(m.rt_ratio - this->rt_validity_.last_accepted_ratio)
                                     : NAN;
        ESP_LOGW(TAG,
                 "RT/RH measurement %lu rejected temperature discarded: REJECT=%s, "
                 "RT/REF=%.6f, step=%s%.6f",
                 static_cast<unsigned long>(m.sequence),
                 rtrh_decoder::reject_reason_to_string(m.reject_reason), m.rt_ratio,
                 std::isfinite(ratio_step) ? "" : "n/a ",
                 std::isfinite(ratio_step) ? ratio_step : 0.0f);
      }
    }

    ESP_LOGW(TAG,
             "RT/RH measurement %lu humidity not published: REJECT=%s "
             "(temperature=%s, quality %.0f%%)%s",
             static_cast<unsigned long>(m.sequence),
             rtrh_decoder::reject_reason_to_string(m.reject_reason),
             m.temperature_valid ? "VALID" : "invalid", m.quality_percent,
             publish_outputs ? "" : " [decode-only A/B]");
    rtrh_decoder::update_latest(m);
    return;
  }

  const float temperature_rate = this->update_thermal_transient_(m.temperature_c);
  m.thermal_transient = this->thermal_.active;
  this->rt_validity_.last_accepted_ratio = m.rt_ratio;
  this->rt_validity_.have_last_accepted_ratio = true;

  ESP_LOGI(TAG, "RT/RH measurement %lu RT: %.3f / REF %.3f us = %.6f -> %.2f C",
           static_cast<unsigned long>(m.sequence), m.rt_period_us, m.ref_period_us,
           m.rt_ratio, m.temperature_c);
  ESP_LOGI(TAG,
           "RT/RH temperature views %lu: air=%s%.2f C, Unni-display=%.2f C",
           static_cast<unsigned long>(m.sequence),
           std::isfinite(m.air_temperature_c) ? "" : "unsupported/",
           std::isfinite(m.air_temperature_c) ? m.air_temperature_c : 0.0f,
           m.display_temperature_c);
  ESP_LOGI(TAG,
           "RT/RH measurement %lu RH: carrier %.3f / REF %.3f us = %.6f -> %.1f %%",
           static_cast<unsigned long>(m.sequence), m.rh_carrier_period_us, m.ref_period_us,
           m.rh_ratio, m.humidity_percent);
  ESP_LOGI(TAG, "RT/RH humidity views %lu: air=%.1f %%, Unni-display=%.1f %%",
           static_cast<unsigned long>(m.sequence), m.humidity_percent,
           m.display_humidity_percent);

  if (this->debug_metrics_) {
    ESP_LOGI(TAG,
             "RT/RH diagnostics %lu: quality %.0f%%, ln(RH carrier/REF)=%.6f, |dT/dt|=%.2f C/min, "
             "thermal_transient=%s, T_extrap=%s, RH_extrap=%s",
             static_cast<unsigned long>(m.sequence), m.quality_percent, m.rh_log,
             temperature_rate, m.thermal_transient ? "YES" : "no",
             m.temperature_extrapolation ? "YES" : "no",
             m.humidity_extrapolation ? "YES" : "no");
  }

  if (publish_outputs) {
    publish(this->out_.ref_period, m.ref_period_us);
    publish(this->out_.rt_period, m.rt_period_us);
    publish(this->out_.rh_state_period, m.rh_state_us);
    publish(this->out_.rt_ratio, m.rt_ratio);
    publish(this->out_.rh_ratio, m.rh_ratio);
    publish(this->out_.rh_log, m.rh_log);
    publish(this->out_.quality, m.quality_percent);
    publish(this->out_.thermal_transient, m.thermal_transient);
    publish(this->out_.temperature_extrapolation, m.temperature_extrapolation);
    publish(this->out_.humidity_extrapolation, m.humidity_extrapolation);
    publish(this->out_.calibration_extrapolation, m.calibration_extrapolation);

#if UNNI_BLE_ENABLED
    // MyAmbience/History expose the physical-air view. Outside the externally
    // validated Air Temperature ratio envelope, retain compatibility by
    // falling back to the RT model rather than publishing NaN.
    const float sensirion_temperature_c =
        std::isfinite(m.air_temperature_c) ? m.air_temperature_c : m.temperature_c;
    sensirion_ble_set_temperature_humidity(sensirion_temperature_c, m.humidity_percent);
#if UNNI_BLE_LIVE_ENABLED
    sensirion_ble_commit_live_advertisement();
#endif
#endif

    this->ha_.temperature = m.temperature_c;
    if (std::isfinite(m.air_temperature_c)) {
      this->ha_.air_temperature = m.air_temperature_c;
      this->ha_.have_air_temperature = true;
    }
    if (std::isfinite(m.display_temperature_c)) {
      this->ha_.display_temperature = m.display_temperature_c;
      this->ha_.have_display_temperature = true;
    }
    this->ha_.humidity = m.humidity_percent;
    if (std::isfinite(m.display_humidity_percent)) {
      this->ha_.display_humidity = m.display_humidity_percent;
      this->ha_.have_display_humidity = true;
    }
    this->ha_.have_temperature = true;
    this->ha_.have_humidity = true;

    if (this->external_powered_()) {
      // USB policy: publish every fresh RT/RH measurement (the Unni cycle is
      // roughly 30 s), rather than repeating cached values on a timer.
      if (this->out_.temperature) this->out_.temperature->publish_state(m.temperature_c);
      if (this->out_.air_temperature && std::isfinite(m.air_temperature_c))
        this->out_.air_temperature->publish_state(m.air_temperature_c);
      if (this->out_.display_temperature && std::isfinite(m.display_temperature_c))
        this->out_.display_temperature->publish_state(m.display_temperature_c);
      if (this->out_.humidity) this->out_.humidity->publish_state(m.humidity_percent);
      if (this->out_.display_humidity && std::isfinite(m.display_humidity_percent))
        this->out_.display_humidity->publish_state(m.display_humidity_percent);
      this->ha_.initial_temperature_published = this->out_.temperature != nullptr;
      this->ha_.initial_air_temperature_published =
          this->out_.air_temperature != nullptr && std::isfinite(m.air_temperature_c);
      this->ha_.initial_display_temperature_published =
          this->out_.display_temperature != nullptr && std::isfinite(m.display_temperature_c);
      this->ha_.initial_humidity_published = this->out_.humidity != nullptr;
      this->ha_.initial_display_humidity_published =
          this->out_.display_humidity != nullptr && std::isfinite(m.display_humidity_percent);
      this->ha_.last_publish_ms = millis();
    } else {
      if (!this->ha_.initial_temperature_published && this->out_.temperature) {
        this->out_.temperature->publish_state(m.temperature_c);
        this->ha_.initial_temperature_published = true;
        this->ha_.last_publish_ms = millis();
      }
      if (!this->ha_.initial_air_temperature_published && this->out_.air_temperature &&
          std::isfinite(m.air_temperature_c)) {
        this->out_.air_temperature->publish_state(m.air_temperature_c);
        this->ha_.initial_air_temperature_published = true;
        this->ha_.last_publish_ms = millis();
      }
      if (!this->ha_.initial_display_temperature_published && this->out_.display_temperature &&
          std::isfinite(m.display_temperature_c)) {
        this->out_.display_temperature->publish_state(m.display_temperature_c);
        this->ha_.initial_display_temperature_published = true;
        this->ha_.last_publish_ms = millis();
      }
      if (!this->ha_.initial_humidity_published && this->out_.humidity) {
        this->out_.humidity->publish_state(m.humidity_percent);
        this->ha_.initial_humidity_published = true;
        this->ha_.last_publish_ms = millis();
      }
      if (!this->ha_.initial_display_humidity_published && this->out_.display_humidity &&
          std::isfinite(m.display_humidity_percent)) {
        this->out_.display_humidity->publish_state(m.display_humidity_percent);
        this->ha_.initial_display_humidity_published = true;
        this->ha_.last_publish_ms = millis();
      }
    }
  } else {
    ESP_LOGI(TAG, "RT/RH decode-only A/B %lu: %.2f C / %.1f %% / quality %.0f%%; no HA/BLE/history publication",
             static_cast<unsigned long>(m.sequence), m.temperature_c, m.humidity_percent, m.quality_percent);
  }

  rtrh_decoder::update_latest(m);
}

void CO2Monitor0601::reset_co2_capture_gate_() {
  this->co2_capture_gate_phase_ = Co2CaptureGatePhase::Active;
  this->co2_window_observations_ = 0;
  this->co2_gate_requested_ = false;
  this->co2_idle_since_ms_ = 0;
  this->co2_guard_since_ms_ = 0;
  i2c_sniffer::set_capture_enabled(true);
}

void CO2Monitor0601::gate_co2_capture_after_window_() {
  if (this->external_powered_() || this->co2_capture_gate_phase_ != Co2CaptureGatePhase::Active)
    return;

  this->co2_gate_requested_ = true;
#if RTRH_DEBUG_CAPTURE
  // Let the small raw capture containing the final useful CO2 frame finish its
  // UDP export before disabling poll(); otherwise debug_export_pending() could
  // remain stuck forever with capture disabled. Production builds skip this.
  if (i2c_sniffer::debug_export_pending()) return;
#endif

  this->co2_gate_requested_ = false;

  // We already have the useful native measurements from this powered CO2
  // window. Stop edge capture before the sensor rail collapses; the slow GPIO
  // decay otherwise creates tens of thousands of meaningless ISR entries and
  // very large debug captures.
  i2c_sniffer::set_capture_enabled(false);
  this->co2_capture_gate_phase_ = Co2CaptureGatePhase::WaitIdleStable;
  this->co2_idle_since_ms_ = 0;
  this->co2_guard_since_ms_ = 0;
  ESP_LOGI(TAG,
           "CO2 measurement window satisfied after %u plausible reading(s): passive I2C sniffer gated until next power-up",
           static_cast<unsigned>(this->co2_window_observations_));
}

void CO2Monitor0601::process_co2_capture_gate_() {
  if (!this->io_initialized_) return;

  if (this->external_powered_()) {
    if (this->co2_capture_gate_phase_ != Co2CaptureGatePhase::Active || power_save::co2_bus_powered_down()) {
      power_save::set_co2_bus_powered_down(false);
      this->reset_co2_capture_gate_();
    }
    return;
  }

  const uint32_t now = millis();
  const bool low_low = i2c_sniffer::bus_is_low_low();

  switch (this->co2_capture_gate_phase_) {
    case Co2CaptureGatePhase::Active: {
      if (this->co2_gate_requested_) {
        this->gate_co2_capture_after_window_();
        if (this->co2_capture_gate_phase_ != Co2CaptureGatePhase::Active) break;
#if RTRH_DEBUG_CAPTURE
        if (i2c_sniffer::debug_export_pending()) break;
#endif
      }

      // Keep the native CO2 window awake while the powered bus is active.
      if (!low_low && !power_save::co2_bus_powered_down())
        power_save::set_co2_bus_powered_down(false);

      // Fallback for a window in which we could not obtain two plausible CO2
      // observations. Once the bus is already quiet LOW/LOW for a full second,
      // the shutdown storm is over and it is safe to use the legacy wake arm.
      const bool quiet_power_down = low_low && i2c_sniffer::last_edge_age_us() >= 1000000U;
      if (quiet_power_down) {
        i2c_sniffer::set_capture_enabled(false);
        power_save::set_co2_bus_powered_down(true);
        this->co2_capture_gate_phase_ = Co2CaptureGatePhase::WakeArmed;
        this->co2_window_observations_ = 0;
        ESP_LOGI(TAG, "CO2 bus quiet fallback: passive sniffer off; GPIO7 wake armed");
      }
      break;
    }

    case Co2CaptureGatePhase::WaitIdleStable:
      if (!low_low) {
        this->co2_idle_since_ms_ = 0;
        break;
      }
      if (this->co2_idle_since_ms_ == 0) {
        this->co2_idle_since_ms_ = now;
        break;
      }
      if (static_cast<uint32_t>(now - this->co2_idle_since_ms_) >= this->co2_wake_idle_stable_ms_) {
        this->co2_capture_gate_phase_ = Co2CaptureGatePhase::Guard;
        this->co2_guard_since_ms_ = now;
        ESP_LOGI(TAG, "CO2 bus LOW/LOW stable for %lu ms; starting %lu ms wake guard",
                 static_cast<unsigned long>(this->co2_wake_idle_stable_ms_),
                 static_cast<unsigned long>(this->co2_wake_guard_time_ms_));
      }
      break;

    case Co2CaptureGatePhase::Guard:
      if (!low_low) {
        this->co2_capture_gate_phase_ = Co2CaptureGatePhase::WaitIdleStable;
        this->co2_idle_since_ms_ = 0;
        this->co2_guard_since_ms_ = 0;
        ESP_LOGD(TAG, "CO2 wake guard reset: bus left LOW/LOW");
        break;
      }
      if (static_cast<uint32_t>(now - this->co2_guard_since_ms_) >= this->co2_wake_guard_time_ms_) {
        power_save::set_co2_bus_powered_down(true);
        this->co2_capture_gate_phase_ = Co2CaptureGatePhase::WakeArmed;
        this->co2_window_observations_ = 0;
        ESP_LOGI(TAG, "CO2 shutdown blanking complete: GPIO7 HIGH wake armed; I2C sniffer remains off");
      }
      break;

    case Co2CaptureGatePhase::WakeArmed:
      // The powered-down bus idles LOW/LOW. Its next power-up raises one or
      // both lines; GPIO7 HIGH wakes the ESP before the useful transaction.
      if (!low_low) {
        power_save::set_co2_bus_powered_down(false);
        i2c_sniffer::set_capture_enabled(true);
        i2c_sniffer::rearm_after_light_sleep();
        this->co2_capture_gate_phase_ = Co2CaptureGatePhase::Active;
        this->co2_window_observations_ = 0;
        this->co2_gate_requested_ = false;
        this->co2_idle_since_ms_ = 0;
        this->co2_guard_since_ms_ = 0;
        ESP_LOGI(TAG, "CO2 bus power-up detected: GPIO7 wake disarmed and passive sniffer re-enabled");
      }
      break;
  }
}

void CO2Monitor0601::process_co2_() {
  static i2c_sniffer::Capture capture;
  if (!i2c_sniffer::poll(capture, co2_decoder::validate_measurement_capture)) return;

  co2_decoder::Result result;
  result.frame_errors = capture.frame_errors;
#if RTRH_DEBUG_CAPTURE
  const char *freeze_reason = capture.frame_errors ? "I2C framing/capture error" : nullptr;
  if (capture.rmt_scl_edges_recovered != 0) {
    ESP_LOGD(TAG, "RMT restored %u missing SCL edge(s) before strict CO2 protocol/CRC validation",
             static_cast<unsigned>(capture.rmt_scl_edges_recovered));
    if (!freeze_reason) freeze_reason = "RMT-restored missing SCL edge";
  }
  if (capture.recovered_missing_clocks != 0) {
    if (capture.recovered_missing_clocks == 1) {
      ESP_LOGD(TAG,
               "Recovered 1 missing SCL clock after strict CO2 protocol/CRC validation (candidate interval=%lu us)",
               static_cast<unsigned long>(capture.recovered_gap_us[0]));
    } else {
      ESP_LOGD(TAG,
               "Recovered 2 missing SCL clocks after strict CO2 protocol/CRC validation (candidate intervals=%lu/%lu us)",
               static_cast<unsigned long>(capture.recovered_gap_us[0]),
               static_cast<unsigned long>(capture.recovered_gap_us[1]));
    }
    if (!freeze_reason) freeze_reason = "software-recovered missing SCL clock";
  }
  if (capture.coalesced_edges_resolved != 0) {
    ESP_LOGD(TAG, "Resolved %u coalesced SDA/SCL edge sample(s)",
             static_cast<unsigned>(capture.coalesced_edges_resolved));
    // A coalesced GPIO sample is expected when ISR latency merges SDA setup
    // with an SCL edge. If framing and protocol decoding remain valid, there
    // is no reason to occupy the single frozen-capture slot with it. Genuine
    // malformed/unhandled frames below still freeze the original waveform.
  }
#endif
  for (uint8_t i = 0; i < capture.frame_count; i++) {
    const auto &frame = capture.frames[i];
    if (!i2c_sniffer::frame_valid(frame)) {
#if RTRH_DEBUG_CAPTURE
      i2c_sniffer::log_frame(frame, "Malformed I2C frame");
      if (!freeze_reason) freeze_reason = "malformed I2C frame";
#endif
      continue;
    }

#if RTRH_DEBUG_CAPTURE
    const uint32_t crc_errors_before = result.crc_errors;
    const uint32_t frame_errors_before = result.frame_errors;
#endif
    if (co2_decoder::process_frame(frame, result)) {
#if RTRH_DEBUG_CAPTURE
      if (result.crc_errors != crc_errors_before) {
        i2c_sniffer::log_frame(frame, "CO2 CRC error frame");
        if (!freeze_reason) freeze_reason = "CO2 CRC error";
      } else if (result.frame_errors != frame_errors_before) {
        i2c_sniffer::log_frame(frame, "Invalid CO2 frame");
        if (!freeze_reason) freeze_reason = "invalid CO2 protocol frame";
      }
#endif
      continue;
    }
#if RTRH_DEBUG_CAPTURE
    i2c_sniffer::log_frame(frame, "Unhandled I2C frame");
    if (!freeze_reason) freeze_reason = "unhandled I2C frame";
#endif
  }
#if RTRH_DEBUG_CAPTURE
  if (freeze_reason)
    i2c_sniffer::freeze_last_capture(capture.debug_raw_sequence, freeze_reason);
#endif

#if UNNI_BLE_HISTORY_ENABLED
  if (result.have_co2 || result.frame_errors != 0)
    sensirion_history_note_co2_capture(capture.raw_scl_edges,
                                       result.frame_errors != 0);
#endif

  if (result.crc_errors) {
    this->co2_.crc_errors += result.crc_errors;
    publish(this->out_.crc_errors, static_cast<float>(this->co2_.crc_errors));
  }
  if (result.frame_errors) {
    this->co2_.frame_errors += result.frame_errors;
    publish(this->out_.frame_errors, static_cast<float>(this->co2_.frame_errors));
  }

  if (result.crc_errors || result.frame_errors) {
    this->co2_.confirmation_required = true;
    this->co2_.have_confirmation_candidate = false;
  }

  if (!result.have_co2) return;

#if UNNI_BLE_HISTORY_ENABLED
  // A protocol- and CRC-valid passive frame is the timing anchor for the
  // cooperative history sender. Plausibility/confirmation remain independent.
  sensirion_history_note_valid_co2_frame();
#endif

  const uint16_t ppm = result.co2_ppm;
  static constexpr uint16_t CO2_MIN_PLAUSIBLE_PPM = 350;
  static constexpr uint16_t CO2_CONFIRM_MAX_DELTA_PPM = 150;

  if (ppm < CO2_MIN_PLAUSIBLE_PPM) {
    ESP_LOGW(TAG,
             "Rejected implausible CO2 reading: %u ppm (< %u ppm); awaiting clean confirmation",
             ppm, CO2_MIN_PLAUSIBLE_PPM);
    this->co2_.confirmation_required = true;
    this->co2_.have_confirmation_candidate = false;
    return;
  }

  if (!this->external_powered_() && this->co2_capture_gate_phase_ == Co2CaptureGatePhase::Active &&
      this->co2_window_observations_ < 0xFF)
    this->co2_window_observations_++;

  if (this->co2_.confirmation_required) {
    if (!this->co2_.have_confirmation_candidate) {
      this->co2_.confirmation_candidate_ppm = ppm;
      this->co2_.have_confirmation_candidate = true;
      ESP_LOGW(TAG,
               "CO2 bus recovery candidate: %u ppm; waiting for a second reading within +/- %u ppm",
               ppm, CO2_CONFIRM_MAX_DELTA_PPM);
      return;
    }

    const int32_t delta = static_cast<int32_t>(ppm) -
                          static_cast<int32_t>(this->co2_.confirmation_candidate_ppm);
    const uint32_t abs_delta = static_cast<uint32_t>(delta < 0 ? -delta : delta);
    if (abs_delta > CO2_CONFIRM_MAX_DELTA_PPM) {
      ESP_LOGW(TAG,
               "CO2 bus recovery candidate changed too much: %u -> %u ppm (delta %lu); restarting confirmation",
               this->co2_.confirmation_candidate_ppm, ppm, static_cast<unsigned long>(abs_delta));
      this->co2_.confirmation_candidate_ppm = ppm;
      return;
    }

    ESP_LOGI(TAG, "CO2 bus recovery confirmed: %u / %u ppm",
             this->co2_.confirmation_candidate_ppm, ppm);
    this->co2_.confirmation_required = false;
    this->co2_.have_confirmation_candidate = false;
  }

  this->accept_co2_ppm_(ppm, "passive sniffer");
}

void CO2Monitor0601::accept_co2_ppm_(uint16_t ppm, const char *source) {
  power_save::on_valid_co2();
#if UNNI_BLE_ENABLED
  sensirion_ble_set_co2(ppm);
#if UNNI_BLE_LIVE_ENABLED
  sensirion_ble_commit_live_advertisement();
#endif
#endif

  this->ha_.co2 = static_cast<float>(ppm);
  this->ha_.have_co2 = true;

  // Two plausible passive observations are enough to identify the end of the
  // useful native CO2 window. Gate before the subsequent rail-collapse storm.
  const bool passive_sniffer = source && std::strcmp(source, "passive sniffer") == 0;
  if (passive_sniffer && !this->external_powered_() && this->co2_window_observations_ >= 2)
    this->gate_co2_capture_after_window_();

  // An explicitly requested active probe is a fresh measurement, so publish it
  // immediately even under the normal battery throttling policy.
  const bool active_probe = source && std::strcmp(source, "active probe") == 0;
  if ((this->external_powered_() || active_probe) && this->out_.co2) {
    this->out_.co2->publish_state(this->ha_.co2);
    this->ha_.initial_co2_published = true;
    this->ha_.last_publish_ms = millis();
  }

  if (this->co2_.have_last_ppm && ppm == this->co2_.last_ppm) {
    ESP_LOGI(TAG, "CO2%s: %u ppm (unchanged)", active_probe ? " [active probe]" : "", ppm);
    return;
  }

  this->co2_.have_last_ppm = true;
  this->co2_.last_ppm = ppm;
  ESP_LOGI(TAG, "CO2%s: %u ppm", active_probe ? " [active probe]" : "", ppm);

  if (!this->external_powered_() && !this->ha_.initial_co2_published && this->out_.co2) {
    this->out_.co2->publish_state(this->ha_.co2);
    this->ha_.initial_co2_published = true;
    this->ha_.last_publish_ms = millis();
  }
}

static uint8_t active_probe_crc_(uint8_t msb, uint8_t lsb) {
  uint8_t crc = 0xFF;
  const uint8_t data[2] = {msb, lsb};
  for (uint8_t value : data) {
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                         : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

void CO2Monitor0601::process_active_i2c_probe_() {
  if (!this->active_i2c_probe_enabled_ || this->external_powered_()) {
    this->active_i2c_probe_phase_ = ActiveProbePhase::Idle;
    return;
  }

  const uint32_t now = millis();

  // While waiting after our own 21B1, restore_passive_gpio_() intentionally
  // resets the capture baseline/last_edge timestamp. Do not mistake that
  // self-generated timestamp for native Unni activity. A non-LOW/LOW bus,
  // however, really means the Unni side woke and we must yield immediately.
  if (this->active_i2c_probe_phase_ == ActiveProbePhase::WaitPeriodic) {
    if (!i2c_sniffer::bus_is_low_low()) {
      ESP_LOGI(TAG, "Active I2C probe cancelled: native CO2 bus became active");
      this->active_i2c_probe_phase_ = ActiveProbePhase::Idle;
      return;
    }
    if (static_cast<int32_t>(now - this->active_i2c_probe_due_ms_) < 0) return;
    this->active_i2c_probe_phase_ = ActiveProbePhase::Idle;
    ESP_LOGI(TAG, "Active I2C probe: periodic-start wait complete; requesting and reading measurement directly");
    uint8_t data[3]{};
    if (!i2c_sniffer::active_read_command(0xEC05, data, sizeof(data))) return;
    if (data[2] != active_probe_crc_(data[0], data[1])) {
      ESP_LOGW(TAG, "Active I2C probe: CO2 CRC mismatch (%02X %02X %02X)",
               data[0], data[1], data[2]);
      return;
    }
    const uint16_t ppm = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (ppm < 350) {
      ESP_LOGW(TAG, "Active I2C probe: rejected implausible CO2 value %u ppm", ppm);
      return;
    }
    this->accept_co2_ppm_(ppm, "active probe");
    return;
  }

  // Idle probes are only started on a genuinely quiet native LOW/LOW bus.
  if (!i2c_sniffer::bus_is_low_low() || i2c_sniffer::last_edge_age_us() < 1000000U)
    return;

  if (this->active_i2c_probe_last_attempt_ms_ != 0 &&
      static_cast<uint32_t>(now - this->active_i2c_probe_last_attempt_ms_) <
          this->active_i2c_probe_interval_ms_)
    return;

  this->active_i2c_probe_last_attempt_ms_ = now;
  ESP_LOGI(TAG,
           "Active I2C probe HARD: bus LOW/LOW and quiet >1 s; driving 3.3 V through 10 kOhm taps and trying EC05");

  // Least invasive first: issue EC05 and perform the response read while we
  // still own the GPIOs and the passive interrupts are disabled. This avoids
  // interpreting our own clock/data transitions as native Unni activity.
  uint8_t data[3]{};
  if (i2c_sniffer::active_read_command(0xEC05, data, sizeof(data))) {
    if (data[2] != active_probe_crc_(data[0], data[1])) {
      ESP_LOGW(TAG, "Active I2C probe: CO2 CRC mismatch (%02X %02X %02X)",
               data[0], data[1], data[2]);
      return;
    }
    const uint16_t ppm = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (ppm < 350) {
      ESP_LOGW(TAG, "Active I2C probe: rejected implausible CO2 value %u ppm", ppm);
      return;
    }
    this->accept_co2_ppm_(ppm, "active probe");
    return;
  }

  // If the direct command/read did not produce a usable transaction, try the exact
  // start command observed from the Unni. If it ACKs, wait one measurement
  // period before EC05. If the bus cannot even be raised or no slave ACKs,
  // this attempt ends immediately and the pins are passive again.
  ESP_LOGI(TAG, "Active I2C probe: EC05 direct read failed; trying observed 21B1 start command");
  if (i2c_sniffer::active_write_command(0x21B1)) {
    this->active_i2c_probe_phase_ = ActiveProbePhase::WaitPeriodic;
    this->active_i2c_probe_due_ms_ = now + 6000U;
    ESP_LOGI(TAG, "Active I2C probe: 21B1 ACK; waiting 6 s for a measurement");
  }
}

#if UNNI_RUNTIME_DIAGNOSTICS
void CO2Monitor0601::runtime_diag_update_max_(uint32_t &slot, uint64_t elapsed_us) {
  const uint32_t value = elapsed_us > 0xFFFFFFFFULL ? 0xFFFFFFFFU : static_cast<uint32_t>(elapsed_us);
  if (value > slot) slot = value;
}

void CO2Monitor0601::runtime_diag_loop_begin_() {
  const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
  const uint32_t now_ms = millis();

  if (this->runtime_diag_.last_loop_start_us != 0) {
    runtime_diag_update_max_(this->runtime_diag_.max_loop_gap_us,
                             now_us - this->runtime_diag_.last_loop_start_us);
  }
  this->runtime_diag_.last_loop_start_us = now_us;
  this->runtime_diag_.current_loop_start_us = now_us;
  this->runtime_diag_.loops++;

  if (this->runtime_diag_.last_heap_report_ms == 0) this->runtime_diag_.last_heap_report_ms = now_ms;
  if (static_cast<uint32_t>(now_ms - this->runtime_diag_.last_heap_report_ms) >= 10000U) {
    this->runtime_diag_.last_heap_report_ms = now_ms;
    ESP_LOGD(TAG, "API diag heap: free=%u B largest_8bit=%u B min_free=%u B",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT)));
  }
}

void CO2Monitor0601::runtime_diag_loop_end_() {
  if (this->runtime_diag_.current_loop_start_us == 0) return;
  const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
  runtime_diag_update_max_(this->runtime_diag_.max_component_us,
                           now_us - this->runtime_diag_.current_loop_start_us);
}
#endif

void CO2Monitor0601::loop() {
#if UNNI_RUNTIME_DIAGNOSTICS
  this->runtime_diag_loop_begin_();
#endif

#if UNNI_SHT43_IDENTITY_PROBE
  static uint32_t last_heap_log_ms = 0;
  const uint32_t now_ms = millis();
  if (last_heap_log_ms == 0 || static_cast<uint32_t>(now_ms - last_heap_log_ms) >= 30000U) {
    last_heap_log_ms = now_ms;
    ESP_LOGI(TAG, "Heap: free=%u B, largest_8bit=%u B",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
  }
#endif
  if (this->sniffer_enabled_ && !this->io_initialized_ && !this->io_initialization_attempted_ &&
      static_cast<uint32_t>(millis() - this->boot_ms_) >= this->start_delay_ms_)
    this->initialize_sniffer_io_();

#if UNNI_RUNTIME_DIAGNOSTICS
  uint64_t stage_us = static_cast<uint64_t>(esp_timer_get_time());
#endif
#if UNNI_BLE_ENABLED
  sensirion_ble_loop();
  sensirion_settings_loop();
#endif
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_loop(this->io_initialized_ ? &sensor_capture_in_progress_ : nullptr);
#endif
#if UNNI_RUNTIME_DIAGNOSTICS
  runtime_diag_update_max_(this->runtime_diag_.max_history_us,
                           static_cast<uint64_t>(esp_timer_get_time()) - stage_us);

  stage_us = static_cast<uint64_t>(esp_timer_get_time());
#endif
  this->process_usb_power_();
  this->process_energy_save_grace_();
#if UNNI_BLE_ENABLED
  this->sync_wifi_ha_from_sensirion_settings_();
#endif
  this->process_wifi_ha_control_();
#if UNNI_BLE_ENABLED
  this->process_ble_pairing_window_();
#endif
#if UNNI_RUNTIME_DIAGNOSTICS
  runtime_diag_update_max_(this->runtime_diag_.max_policy_us,
                           static_cast<uint64_t>(esp_timer_get_time()) - stage_us);

  stage_us = static_cast<uint64_t>(esp_timer_get_time());
#endif
  this->maybe_publish_ha_();
#if UNNI_RUNTIME_DIAGNOSTICS
  runtime_diag_update_max_(this->runtime_diag_.max_ha_publish_us,
                           static_cast<uint64_t>(esp_timer_get_time()) - stage_us);

  stage_us = static_cast<uint64_t>(esp_timer_get_time());
#endif
  this->process_battery_();
#if UNNI_RUNTIME_DIAGNOSTICS
  runtime_diag_update_max_(this->runtime_diag_.max_battery_us,
                           static_cast<uint64_t>(esp_timer_get_time()) - stage_us);
#endif

  if (!this->io_initialized_) {
#if UNNI_RUNTIME_DIAGNOSTICS
    this->runtime_diag_loop_end_();
#endif
    return;
  }

  const bool rtrh_power_path = this->rtrh_enabled_ || this->rtrh_gpio_setup_ ||
                                this->rtrh_edge_capture_ || this->rtrh_decode_only_;

  // The first RT/RH edge after automatic Light-sleep opens the battery awake
  // window from ISR context. Restore the passive CO2 GPIO input/interrupt state
  // here in normal task context before polling the next I2C transaction.
  const uint32_t wake_generation = power_save::wake_generation();
  if (wake_generation != this->light_sleep_wake_generation_) {
    this->light_sleep_wake_generation_ = wake_generation;
    rtrh_decoder::rearm_after_light_sleep();
    i2c_sniffer::rearm_after_light_sleep();
  }

  // Battery CO2 gating: stop passive edge capture as soon as the useful native
  // measurement window is complete, ignore the analog rail-collapse storm,
  // then arm GPIO7 only after the dead LOW/LOW bus has been stable plus a guard
  // interval. USB keeps the sniffer continuously active.
  this->process_co2_capture_gate_();

#if UNNI_RUNTIME_DIAGNOSTICS
  stage_us = static_cast<uint64_t>(esp_timer_get_time());
#endif
  if (this->rtrh_enabled_) {
    this->process_rtrh_();
  } else if (this->rtrh_decode_only_) {
    this->process_rtrh_(false);
  } else if (this->rtrh_edge_capture_) {
    // A/B test: run the real RT/RH edge ISR and its capture state machine, but
    // do not publish or feed measurements into HA/BLE. Poll only to release
    // completed snapshots and keep the capture path representative.
    rtrh_decoder::loop();
    rtrh_decoder::Measurement discarded;
    if (rtrh_decoder::poll(discarded)) {
      power_save::on_rtrh_complete();
#if UNNI_BLE_HISTORY_ENABLED
      sensirion_history_note_rtrh_cycle();
#endif
      ESP_LOGI(TAG, "RT/RH ISR-only capture %lu complete: %s, quality %.0f%%",
               static_cast<unsigned long>(discarded.sequence),
               discarded.valid ? "VALID" : "REJECT", discarded.quality_percent);
    }
  }
#if UNNI_RUNTIME_DIAGNOSTICS
  runtime_diag_update_max_(this->runtime_diag_.max_rtrh_us,
                           static_cast<uint64_t>(esp_timer_get_time()) - stage_us);

  stage_us = static_cast<uint64_t>(esp_timer_get_time());
#endif
  this->process_co2_();
  this->process_active_i2c_probe_();
  i2c_sniffer::log_edge_diagnostics(millis());
#if UNNI_RUNTIME_DIAGNOSTICS
  runtime_diag_update_max_(this->runtime_diag_.max_co2_us,
                           static_cast<uint64_t>(esp_timer_get_time()) - stage_us);

  stage_us = static_cast<uint64_t>(esp_timer_get_time());
#endif
#if RTRH_DEBUG_CAPTURE
  power_save::set_transport_busy(i2c_sniffer::debug_export_pending() ||
                                 rtrh_decoder::debug_export_pending());
#else
  power_save::set_transport_busy(false);
#endif
  if (rtrh_power_path) power_save::loop();
#if UNNI_RUNTIME_DIAGNOSTICS
  runtime_diag_update_max_(this->runtime_diag_.max_power_save_us,
                           static_cast<uint64_t>(esp_timer_get_time()) - stage_us);

  this->runtime_diag_loop_end_();
#endif
}

}  // namespace co2_monitor_0601
}  // namespace esphome
