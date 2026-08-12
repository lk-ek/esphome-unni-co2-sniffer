// SPDX-License-Identifier: GPL-3.0-or-later
#include "bus_sniffer.h"

#include "ble_options.h"
#include "co2_decoder.h"
#include "rtrh_decoder.h"
#include "power_save.h"

#if UNNI_BLE_ENABLED
#include "sensirion_ble.h"
#endif
#if UNNI_BLE_HISTORY_ENABLED
#include "sensirion_history.h"
#endif

#include "driver/gpio.h"
#include "esphome/core/log.h"

#include <cmath>
#include <string>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "bus_sniffer";

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
void BusSniffer::configure_gatt_server(esp32_ble_server::BLEServer *server) {
  // The BLE server is auto-loaded by the component, so reproduce the Gadget
  // Device Information identity that previously lived in user YAML.
  if (server != nullptr) {
    auto *info = server->get_service(esp32_ble::ESPBTUUID::from_uint16(0x180A));
    if (info != nullptr) {
      if (auto *manufacturer = info->get_characteristic(0x2A29))
        manufacturer->set_value(std::string("Sensirion"));
      if (auto *model = info->get_characteristic(0x2A24))
        model->set_value(std::string("MyCO2 Gadget"));
      if (auto *firmware = info->get_characteristic(0x2A26))
        firmware->set_value(std::string("1.0.1"));
    }
  }
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_configure_gatt(server);
#endif
}

void BusSniffer::set_ble_advertising_interval(uint32_t interval_ms) {
  sensirion_ble_set_advertising_interval(interval_ms);
}

void BusSniffer::gap_event_handler(esp_gap_ble_cb_event_t event,
                                   esp_ble_gap_cb_param_t *param) {
  sensirion_ble_gap_event_handler(event, param);
}

void BusSniffer::gatts_event_handler(esp_gatts_cb_event_t event,
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

bool BusSniffer::initialize_sniffer_io_() {
  if (this->io_initialized_) return true;

  // Both decoder modules use the same ESP-IDF GPIO ISR service.
  const esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "gpio_install_isr_service failed: %d", err);
    return false;
  }

  if (!co2_decoder::setup(this->co2_sda_pin_, this->co2_scl_pin_)) {
    ESP_LOGE(TAG, "CO2 decoder GPIO/ISR setup failed");
    return false;
  }
  if (!rtrh_decoder::setup(this->rtrh_g10_pin_, this->rtrh_g13_pin_)) {
    ESP_LOGE(TAG, "RT/RH decoder GPIO/ISR setup failed");
    return false;
  }

  this->io_initialized_ = true;
  if (!power_save::setup(this->light_sleep_enabled_, this->light_sleep_max_awake_ms_,
                         this->rtrh_g10_pin_, this->rtrh_g13_pin_,
                         this->co2_sda_pin_, this->co2_scl_pin_)) {
    ESP_LOGW(TAG, "Requested auto Light-sleep could not be enabled; continuing normally");
  } else if (power_save::enabled()) {
    // CO2 traffic must not wake the chip or leave partial I2C transactions
    // behind while the CPU is sleeping. It is enabled only in an RT/RH window.
    co2_decoder::set_capture_enabled(false);
  }
  ESP_LOGI(TAG, "Sniffer GPIO/ISR initialization enabled after %lu ms",
           static_cast<unsigned long>(millis() - this->boot_ms_));
  return true;
}

void BusSniffer::setup() {
#if UNNI_BLE_ENABLED
  sensirion_ble_setup();
#endif
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_setup();
#endif

  this->boot_ms_ = millis();
  if (this->start_delay_ms_ == 0) {
    this->initialize_sniffer_io_();
  } else {
    ESP_LOGI(TAG, "Sniffer GPIO isolation active for first %lu ms; signal pins untouched",
             static_cast<unsigned long>(this->start_delay_ms_));
  }

#if RTRH_DEBUG_CAPTURE
  co2_decoder::register_debug_handler();
  rtrh_decoder::register_debug_handlers();
  ESP_LOGD(TAG, "Raw debug: /capture, /rt_rh_capture.csv, /rt_rh_timing.csv");
#else
  ESP_LOGD(TAG, "RT/RH time-phase decoder active; debug capture disabled");
#endif

  publish(this->out_.crc_errors, 0.0f);
  publish(this->out_.frame_errors, 0.0f);
  ESP_LOGI(TAG, "Passive CO2 + RT/RH sniffer ready");
}

void BusSniffer::maybe_publish_ha_() {
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

float BusSniffer::update_thermal_transient_(float temperature_c) {
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

void BusSniffer::process_rtrh_() {
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

  rtrh_decoder::update_latest(m);
}

void BusSniffer::process_co2_() {
  co2_decoder::Result result;
  if (!co2_decoder::poll(result)) return;

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

  if (this->co2_.have_last_ppm && ppm == this->co2_.last_ppm) {
    ESP_LOGV(TAG, "CO2 unchanged: %u ppm", ppm);
    return;
  }

  this->co2_.have_last_ppm = true;
  this->co2_.last_ppm = ppm;
  ESP_LOGI(TAG, "CO2: %u ppm", ppm);

  if (!this->ha_.initial_co2_published && this->out_.co2) {
    this->out_.co2->publish_state(this->ha_.co2);
    this->ha_.initial_co2_published = true;
    this->ha_.last_publish_ms = millis();
  }
}

void BusSniffer::loop() {
  if (!this->io_initialized_ &&
      static_cast<uint32_t>(millis() - this->boot_ms_) >= this->start_delay_ms_)
    this->initialize_sniffer_io_();

#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_loop();
#endif
  this->maybe_publish_ha_();

  if (!this->io_initialized_) return;

  if (power_save::enabled())
    co2_decoder::set_capture_enabled(power_save::awake_window_active());

  this->process_rtrh_();
  this->process_co2_();
  power_save::loop();

  // power_save::loop() may have just closed the window. Drop any partial CO2
  // transaction immediately instead of carrying it into the next sleep cycle.
  if (power_save::enabled())
    co2_decoder::set_capture_enabled(power_save::awake_window_active());
}

}  // namespace bus_sniffer
}  // namespace esphome
