// SPDX-License-Identifier: GPL-3.0-or-later
#include "bus_sniffer.h"

#include "ble_options.h"
#include "calibration.h"
#include "co2_decoder.h"
#include "rtrh_decoder.h"

#if UNNI_BLE_ENABLED
#include "sensirion_ble.h"
#endif
#if UNNI_BLE_HISTORY_ENABLED
#include "sensirion_history.h"
#endif

#include "driver/gpio.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "bus_sniffer";

#if UNNI_BLE_ENABLED
void BusSniffer::configure_gatt_server(esp32_ble_server::BLEServer *server) {
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_configure_gatt(server);
#else
  (void) server;
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
  if (this->sniffer_io_initialized_) return true;

  // Both decoders share the ESP-IDF GPIO ISR service.
  esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "gpio_install_isr_service failed: %d", err);
    return false;
  }

  if (!co2_decoder::setup()) {
    ESP_LOGE(TAG, "CO2 decoder GPIO/ISR setup failed");
    return false;
  }
  if (!rtrh_decoder::setup()) {
    ESP_LOGE(TAG, "RT/RH decoder GPIO/ISR setup failed");
    return false;
  }

  this->sniffer_io_initialized_ = true;
  ESP_LOGI(TAG, "Sniffer GPIO/ISR initialization enabled after %lu ms",
           static_cast<unsigned long>(millis() - this->sniffer_boot_ms_));
  return true;
}

void BusSniffer::setup() {
#if UNNI_BLE_ENABLED
  sensirion_ble_setup();
#endif
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_setup();
#endif

  this->sniffer_boot_ms_ = millis();
  if (this->sniffer_start_delay_ms_ == 0) {
    this->initialize_sniffer_io_();
  } else {
    ESP_LOGI(TAG, "Sniffer GPIO isolation active for first %lu ms; signal pins untouched",
             static_cast<unsigned long>(this->sniffer_start_delay_ms_));
  }

#if RTRH_DEBUG_CAPTURE
  co2_decoder::register_debug_handler();
  rtrh_decoder::register_debug_handlers();
  ESP_LOGD(TAG, "Raw debug: /capture, /rt_rh_capture.csv, /rt_rh_timing.csv");
#else
  ESP_LOGD(TAG, "RT/RH time-phase decoder active; debug capture disabled");
#endif

  if (this->crc_errors_sensor_) this->crc_errors_sensor_->publish_state(0);
  if (this->frame_errors_sensor_) this->frame_errors_sensor_->publish_state(0);

  ESP_LOGI(TAG, "Passive CO2 + RT/RH sniffer ready");
}

void BusSniffer::maybe_publish_ha_() {
  const uint32_t now = millis();
  if (this->last_ha_publish_ms_ &&
      static_cast<uint32_t>(now - this->last_ha_publish_ms_) < this->ha_publish_interval_ms_)
    return;

  bool published = false;
  if (this->ha_have_co2_ && this->co2_sensor_ && this->ha_initial_co2_published_) {
    this->co2_sensor_->publish_state(this->ha_co2_);
    published = true;
  }
  if (this->ha_have_temperature_ && this->rt_temperature_sensor_ &&
      this->ha_initial_temperature_published_) {
    this->rt_temperature_sensor_->publish_state(this->ha_temperature_);
    published = true;
  }
  if (this->ha_have_humidity_ && this->rh_humidity_sensor_ &&
      this->ha_initial_humidity_published_) {
    this->rh_humidity_sensor_->publish_state(this->ha_humidity_);
    published = true;
  }
  if (published) this->last_ha_publish_ms_ = now;
}

void BusSniffer::loop() {
  if (!this->sniffer_io_initialized_ &&
      static_cast<uint32_t>(millis() - this->sniffer_boot_ms_) >= this->sniffer_start_delay_ms_)
    this->initialize_sniffer_io_();

#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_loop();
#endif
  this->maybe_publish_ha_();

  if (!this->sniffer_io_initialized_) return;

  // RT/RH decoder returns one complete, calibrated measurement per sensor cycle.
  rtrh_decoder::loop();
  rtrh_decoder::Measurement m;
  if (rtrh_decoder::poll(m)) {
    ESP_LOGI(TAG,
             "RT/RH measurement %lu quality: REF %.3f us / %.3f ms / %u, "
             "RT %.3f us / %.3f ms / %u, RH %.3f ms / state %.3f us (%u/%lu) -> %s",
             static_cast<unsigned long>(m.sequence), m.ref_period_us, m.ref_duration_ms,
             static_cast<unsigned>(m.ref_count), m.rt_phase_period_us, m.rt_duration_ms,
             static_cast<unsigned>(m.rt_phase_count), m.rh_duration_ms, m.rh_state_us,
             static_cast<unsigned>(m.rh_state_samples), static_cast<unsigned long>(m.rh_state_seen),
             m.valid ? "VALID" : "REJECT");

    if (!m.valid) {
      if (this->measurement_quality_sensor_) this->measurement_quality_sensor_->publish_state(m.quality_percent);
      if (this->ref_period_sensor_) this->ref_period_sensor_->publish_state(m.ref_period_us);
      if (this->rt_period_sensor_ && m.rt_period_us > 0.0f) this->rt_period_sensor_->publish_state(m.rt_period_us);
      if (this->rh_state_period_sensor_ && m.rh_state_us > 0.0f) this->rh_state_period_sensor_->publish_state(m.rh_state_us);
      if (this->rt_ratio_sensor_ && std::isfinite(m.rt_ratio)) this->rt_ratio_sensor_->publish_state(m.rt_ratio);
      if (this->rh_ratio_sensor_ && std::isfinite(m.rh_ratio)) this->rh_ratio_sensor_->publish_state(m.rh_ratio);
      if (this->temperature_extrapolation_sensor_) this->temperature_extrapolation_sensor_->publish_state(true);
      if (this->humidity_extrapolation_sensor_) this->humidity_extrapolation_sensor_->publish_state(true);
      if (this->calibration_extrapolation_sensor_) this->calibration_extrapolation_sensor_->publish_state(true);

      ESP_LOGW(TAG, "RT/RH measurement %lu values not published: REJECT=%s (quality %.0f%%)",
               static_cast<unsigned long>(m.sequence),
               rtrh_decoder::reject_reason_to_string(m.reject_reason), m.quality_percent);
    } else {
      const uint32_t now_ms = millis();
      float temperature_rate = 0.0f;
      if (this->have_last_valid_temperature_) {
        const uint32_t dt_ms = now_ms - this->last_valid_temperature_ms_;
        if (dt_ms)
          temperature_rate = fabsf(m.temperature_c - this->last_valid_temperature_c_) *
                             60000.0f / float(dt_ms);
      }
      if (!this->thermal_transient_active_) {
        if (this->have_last_valid_temperature_ &&
            temperature_rate >= this->thermal_transient_on_rate_c_per_min_)
          this->thermal_transient_active_ = true;
      } else if (temperature_rate <= this->thermal_transient_off_rate_c_per_min_) {
        this->thermal_transient_active_ = false;
      }
      this->last_valid_temperature_c_ = m.temperature_c;
      this->last_valid_temperature_ms_ = now_ms;
      this->have_last_valid_temperature_ = true;
      m.thermal_transient = this->thermal_transient_active_;

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

      if (this->ref_period_sensor_) this->ref_period_sensor_->publish_state(m.ref_period_us);
      if (this->rt_period_sensor_) this->rt_period_sensor_->publish_state(m.rt_period_us);
      if (this->rh_state_period_sensor_) this->rh_state_period_sensor_->publish_state(m.rh_state_us);
      if (this->rt_ratio_sensor_) this->rt_ratio_sensor_->publish_state(m.rt_ratio);
      if (this->rh_ratio_sensor_) this->rh_ratio_sensor_->publish_state(m.rh_ratio);
      if (this->rh_log_sensor_) this->rh_log_sensor_->publish_state(m.rh_log);
      if (this->measurement_quality_sensor_) this->measurement_quality_sensor_->publish_state(m.quality_percent);
      if (this->thermal_transient_sensor_) this->thermal_transient_sensor_->publish_state(m.thermal_transient);
      if (this->temperature_extrapolation_sensor_) this->temperature_extrapolation_sensor_->publish_state(m.temperature_extrapolation);
      if (this->humidity_extrapolation_sensor_) this->humidity_extrapolation_sensor_->publish_state(m.humidity_extrapolation);
      if (this->calibration_extrapolation_sensor_) this->calibration_extrapolation_sensor_->publish_state(m.calibration_extrapolation);

#if UNNI_BLE_ENABLED
      sensirion_ble_set_temperature_humidity(m.temperature_c, m.humidity_percent);
#if UNNI_BLE_LIVE_ENABLED
      sensirion_ble_commit_live_advertisement();
#endif
#endif

      this->ha_temperature_ = m.temperature_c;
      this->ha_humidity_ = m.humidity_percent;
      this->ha_have_temperature_ = true;
      this->ha_have_humidity_ = true;
      if (!this->ha_initial_temperature_published_ && this->rt_temperature_sensor_) {
        this->rt_temperature_sensor_->publish_state(m.temperature_c);
        this->ha_initial_temperature_published_ = true;
        this->last_ha_publish_ms_ = millis();
      }
      if (!this->ha_initial_humidity_published_ && this->rh_humidity_sensor_) {
        this->rh_humidity_sensor_->publish_state(m.humidity_percent);
        this->ha_initial_humidity_published_ = true;
        this->last_ha_publish_ms_ = millis();
      }
    }
    rtrh_decoder::update_latest(m);
  }

  // CO2 capture/decode is independent and produces at most one result here.
  co2_decoder::Result co2;
  if (co2_decoder::poll(co2)) {
    if (co2.crc_errors) {
      this->crc_errors_ += co2.crc_errors;
      if (this->crc_errors_sensor_) this->crc_errors_sensor_->publish_state(this->crc_errors_);
    }
    if (co2.frame_errors) {
      this->frame_errors_ += co2.frame_errors;
      if (this->frame_errors_sensor_) this->frame_errors_sensor_->publish_state(this->frame_errors_);
    }
    if (co2.have_co2) {
      const uint16_t ppm = co2.co2_ppm;
#if UNNI_BLE_ENABLED
      sensirion_ble_set_co2(ppm);
#endif
      this->ha_co2_ = static_cast<float>(ppm);
      this->ha_have_co2_ = true;
      if (!this->have_last_ppm_ || ppm != this->last_ppm_) {
        this->have_last_ppm_ = true;
        this->last_ppm_ = ppm;
        ESP_LOGI(TAG, "CO2: %u ppm", ppm);
        if (!this->ha_initial_co2_published_ && this->co2_sensor_) {
          this->co2_sensor_->publish_state(this->ha_co2_);
          this->ha_initial_co2_published_ = true;
          this->last_ha_publish_ms_ = millis();
        }
      } else {
        ESP_LOGV(TAG, "CO2 unchanged: %u ppm", ppm);
      }
    }
  }
}

}  // namespace bus_sniffer
}  // namespace esphome
