// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "co2_monitor_0601.h"

#include "ble_options.h"
#include "co2_decoder.h"
#include "i2c_sniffer.h"
#include "rtrh_decoder.h"
#include "power_save.h"

#if UNNI_BLE_ENABLED
#include "sensirion_ble.h"
#endif
#if UNNI_BLE_HISTORY_ENABLED
#include "sensirion_history.h"
#endif

#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esphome/core/log.h"

#include <cmath>
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

}  // namespace

#if UNNI_BLE_ENABLED
void CO2Monitor0601::set_ble_advertising_interval(uint32_t interval_ms) {
  this->ble_usb_advertising_interval_ms_ = interval_ms;
  sensirion_ble_set_advertising_interval(interval_ms);
}

void CO2Monitor0601::gap_event_handler(esp_gap_ble_cb_event_t event,
                                   esp_ble_gap_cb_param_t *param) {
  sensirion_ble_gap_event_handler(event, param);
}

void CO2Monitor0601::gatts_event_handler(esp_gatts_cb_event_t event,
                                     esp_gatt_if_t gatts_if,
                                     esp_ble_gatts_cb_param_t *param) {
  sensirion_ble_gatts_event_handler(event, param);
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
  ESP_LOGI(TAG, "Energy Save Mode: %s", enabled ? "ON (battery policy forced)" : "OFF (automatic USB/battery policy)");
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

  if (this->io_initialized_ && power_save::enabled())
    i2c_sniffer::set_capture_enabled(external || power_save::awake_window_active());

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
           this->energy_save_mode_ ? " (Energy Save Mode override)" : "");
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

  const bool raw = gpio_get_level(static_cast<gpio_num_t>(this->usb_power_.pin)) != 0;
  const uint32_t now = millis();
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

  // Keep the physical USB entity truthful, but let Energy Save Mode override
  // the runtime policy so USB power meters can measure the same behavior used
  // on battery.
  this->apply_power_policy_();

  // Cell voltage is not a useful open-circuit SOC estimate while USB is
  // present and the battery node may be driven by the charger. Mark Battery
  // Level unavailable until the device is back on battery power.
  if (raw && this->out_.battery_level)
    this->out_.battery_level->publish_state(NAN);

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
  } else {
    const float battery_level = battery_percent_from_voltage_(battery_voltage);
    publish(this->out_.battery_level, battery_level);
    ESP_LOGD(TAG, "Battery: %.3f V -> %.0f %%", battery_voltage, battery_level);
  }
}

bool CO2Monitor0601::initialize_sniffer_io_() {
  if (this->io_initialized_) return true;

  if (!i2c_sniffer::setup(this->co2_sda_pin_, this->co2_scl_pin_)) {
    ESP_LOGE(TAG, "I2C sniffer GPIO/ISR setup failed");
    return false;
  }
  if (!rtrh_decoder::setup(this->rt_pin_, this->rh_pin_)) {
    ESP_LOGE(TAG, "RT/RH decoder GPIO/ISR setup failed");
    return false;
  }

  this->io_initialized_ = true;
  if (!power_save::setup(this->light_sleep_enabled_, this->light_sleep_max_awake_ms_,
                         this->rt_pin_, this->rh_pin_,
                         this->co2_sda_pin_, this->co2_scl_pin_)) {
    ESP_LOGW(TAG, "Requested auto Light-sleep could not be enabled; continuing normally");
  } else if (power_save::enabled()) {
    // If VBUS was already debounced before delayed sniffer initialization,
    // immediately apply the matching USB/battery power policy.
    this->apply_power_policy_(true);
    i2c_sniffer::set_capture_enabled(this->external_powered_() || power_save::awake_window_active());
  }
  ESP_LOGI(TAG, "Sniffer GPIO/ISR initialization enabled after %lu ms",
           static_cast<unsigned long>(millis() - this->boot_ms_));
  return true;
}

void CO2Monitor0601::setup() {
  if (this->energy_save_switch_ != nullptr) this->energy_save_switch_->publish_state(this->energy_save_mode_);

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
      if (auto *manufacturer = info->get_characteristic(0x2A29))
        manufacturer->set_value(std::string("Gadget"));
      if (auto *model = info->get_characteristic(0x2A24))
        model->set_value(std::string("MyCO2 Gadget"));
      if (auto *firmware = info->get_characteristic(0x2A26))
        firmware->set_value(std::string("1.0.1"));
    }
#if UNNI_BLE_HISTORY_ENABLED
    sensirion_history_configure_gatt(this->gatt_server_);
#endif
  } else {
    ESP_LOGE(TAG, "BLE enabled but no GATT server instance is available");
  }
#endif
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_setup();
#endif

  this->setup_usb_power_();
  this->setup_battery_adc_();
  this->boot_ms_ = millis();
  if (this->start_delay_ms_ == 0) {
    this->initialize_sniffer_io_();
  } else {
    ESP_LOGI(TAG, "Sniffer GPIO isolation active for first %lu ms; signal pins untouched",
             static_cast<unsigned long>(this->start_delay_ms_));
  }

#if RTRH_DEBUG_CAPTURE
  i2c_sniffer::register_debug_handler();
  rtrh_decoder::register_debug_handlers();
  ESP_LOGD(TAG, "Raw debug: /capture, /rt_rh_capture.csv, /rt_rh_timing.csv");
#else
  ESP_LOGD(TAG, "RT/RH time-phase decoder active; debug capture disabled");
#endif

  publish(this->out_.crc_errors, 0.0f);
  publish(this->out_.frame_errors, 0.0f);
  ESP_LOGI(TAG, "Passive CO2 + RT/RH sniffer ready");
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
  if (this->ha_.have_humidity && this->out_.humidity) {
    this->out_.humidity->publish_state(this->ha_.humidity);
    this->ha_.initial_humidity_published = true;
    published = true;
  }
  if (published) this->ha_.last_publish_ms = millis();
}

void CO2Monitor0601::maybe_publish_ha_() {
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
  if (this->ha_.have_humidity && this->out_.humidity &&
      this->ha_.initial_humidity_published) {
    this->out_.humidity->publish_state(this->ha_.humidity);
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

void CO2Monitor0601::process_rtrh_() {
  rtrh_decoder::loop();

  rtrh_decoder::Measurement m;
  if (!rtrh_decoder::poll(m)) return;
  power_save::on_rtrh_complete(m.valid);

  ESP_LOGI(TAG,
           "RT/RH measurement %lu quality: REF %.3f us / %.3f ms / %u, "
           "RT %.3f us / %.3f ms / %u, RH %.3f ms / state %.3f us (%u/%lu) -> %s",
           static_cast<unsigned long>(m.sequence), m.ref_period_us, m.ref_duration_ms,
           static_cast<unsigned>(m.ref_count), m.rt_phase_period_us, m.rt_duration_ms,
           static_cast<unsigned>(m.rt_phase_count), m.rh_duration_ms, m.rh_state_us,
           static_cast<unsigned>(m.rh_state_samples), static_cast<unsigned long>(m.rh_state_seen),
           m.valid ? "VALID" : "REJECT");

  if (!m.valid) {
    publish(this->out_.quality, m.quality_percent);
    publish(this->out_.ref_period, m.ref_period_us);
    publish_positive(this->out_.rt_period, m.rt_period_us);
    publish_positive(this->out_.rh_state_period, m.rh_state_us);
    publish_finite(this->out_.rt_ratio, m.rt_ratio);
    publish_finite(this->out_.rh_ratio, m.rh_ratio);
    publish(this->out_.temperature_extrapolation, true);
    publish(this->out_.humidity_extrapolation, true);
    publish(this->out_.calibration_extrapolation, true);

    ESP_LOGW(TAG, "RT/RH measurement %lu values not published: REJECT=%s (quality %.0f%%)",
             static_cast<unsigned long>(m.sequence),
             rtrh_decoder::reject_reason_to_string(m.reject_reason), m.quality_percent);
    rtrh_decoder::update_latest(m);
    return;
  }

  const float temperature_rate = this->update_thermal_transient_(m.temperature_c);
  m.thermal_transient = this->thermal_.active;

  ESP_LOGI(TAG, "RT/RH measurement %lu RT: %.3f / REF %.3f us = %.6f -> %.2f C",
           static_cast<unsigned long>(m.sequence), m.rt_period_us, m.ref_period_us,
           m.rt_ratio, m.temperature_c);
  ESP_LOGI(TAG, "RT/RH measurement %lu RH: state %.3f / REF %.3f us = %.6f -> %.1f %%",
           static_cast<unsigned long>(m.sequence), m.rh_state_us, m.ref_period_us,
           m.rh_ratio, m.humidity_percent);

  if (this->debug_metrics_) {
    ESP_LOGI(TAG,
             "RT/RH diagnostics %lu: quality %.0f%%, ln(RH/REF)=%.6f, |dT/dt|=%.2f C/min, "
             "thermal_transient=%s, T_extrap=%s, RH_extrap=%s",
             static_cast<unsigned long>(m.sequence), m.quality_percent, m.rh_log,
             temperature_rate, m.thermal_transient ? "YES" : "no",
             m.temperature_extrapolation ? "YES" : "no",
             m.humidity_extrapolation ? "YES" : "no");
  }

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
  sensirion_ble_set_temperature_humidity(m.temperature_c, m.humidity_percent);
#if UNNI_BLE_LIVE_ENABLED
  sensirion_ble_commit_live_advertisement();
#endif
#endif

  this->ha_.temperature = m.temperature_c;
  this->ha_.humidity = m.humidity_percent;
  this->ha_.have_temperature = true;
  this->ha_.have_humidity = true;

  if (this->external_powered_()) {
    // USB policy: publish every fresh RT/RH measurement (the Unni cycle is
    // roughly 30 s), rather than repeating cached values on a timer.
    if (this->out_.temperature) this->out_.temperature->publish_state(m.temperature_c);
    if (this->out_.humidity) this->out_.humidity->publish_state(m.humidity_percent);
    this->ha_.initial_temperature_published = this->out_.temperature != nullptr;
    this->ha_.initial_humidity_published = this->out_.humidity != nullptr;
    this->ha_.last_publish_ms = millis();
  } else {
    if (!this->ha_.initial_temperature_published && this->out_.temperature) {
      this->out_.temperature->publish_state(m.temperature_c);
      this->ha_.initial_temperature_published = true;
      this->ha_.last_publish_ms = millis();
    }
    if (!this->ha_.initial_humidity_published && this->out_.humidity) {
      this->out_.humidity->publish_state(m.humidity_percent);
      this->ha_.initial_humidity_published = true;
      this->ha_.last_publish_ms = millis();
    }
  }

  rtrh_decoder::update_latest(m);
}

void CO2Monitor0601::process_co2_() {
  static i2c_sniffer::Capture capture;
  if (!i2c_sniffer::poll(capture, co2_decoder::validate_measurement_capture)) return;

  co2_decoder::Result result;
  result.frame_errors = capture.frame_errors;
#if RTRH_DEBUG_CAPTURE
  const char *freeze_reason = capture.frame_errors ? "I2C framing/capture error" : nullptr;
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

  if (result.crc_errors) {
    this->co2_.crc_errors += result.crc_errors;
    publish(this->out_.crc_errors, static_cast<float>(this->co2_.crc_errors));
  }
  if (result.frame_errors) {
    this->co2_.frame_errors += result.frame_errors;
    publish(this->out_.frame_errors, static_cast<float>(this->co2_.frame_errors));
  }
  if (!result.have_co2) return;
  power_save::on_valid_co2();

  const uint16_t ppm = result.co2_ppm;
#if UNNI_BLE_ENABLED
  sensirion_ble_set_co2(ppm);
#endif

  this->ha_.co2 = static_cast<float>(ppm);
  this->ha_.have_co2 = true;

  if (this->external_powered_() && this->out_.co2) {
    // USB policy: every valid CO2 frame is a useful fresh measurement, even
    // when the integer ppm value happens to be unchanged.
    this->out_.co2->publish_state(this->ha_.co2);
    this->ha_.initial_co2_published = true;
    this->ha_.last_publish_ms = millis();
  }

  if (this->co2_.have_last_ppm && ppm == this->co2_.last_ppm) {
    ESP_LOGV(TAG, "CO2 unchanged: %u ppm", ppm);
    return;
  }

  this->co2_.have_last_ppm = true;
  this->co2_.last_ppm = ppm;
  ESP_LOGI(TAG, "CO2: %u ppm", ppm);

  if (!this->external_powered_() && !this->ha_.initial_co2_published && this->out_.co2) {
    this->out_.co2->publish_state(this->ha_.co2);
    this->ha_.initial_co2_published = true;
    this->ha_.last_publish_ms = millis();
  }
}

void CO2Monitor0601::loop() {
  if (!this->io_initialized_ &&
      static_cast<uint32_t>(millis() - this->boot_ms_) >= this->start_delay_ms_)
    this->initialize_sniffer_io_();

#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_loop();
#endif
  this->process_usb_power_();
  this->maybe_publish_ha_();
  this->process_battery_();

  if (!this->io_initialized_) return;

  if (power_save::enabled())
    i2c_sniffer::set_capture_enabled(this->external_powered_() || power_save::awake_window_active());

  this->process_rtrh_();
  this->process_co2_();
  power_save::loop();

  // power_save::loop() may have just closed the window. Drop any partial CO2
  // transaction immediately instead of carrying it into the next sleep cycle.
  if (power_save::enabled())
    i2c_sniffer::set_capture_enabled(this->external_powered_() || power_save::awake_window_active());
}

}  // namespace co2_monitor_0601
}  // namespace esphome
