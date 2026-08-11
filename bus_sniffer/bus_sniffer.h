#pragma once

#include "ble_options.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#if UNNI_BLE_ENABLED
#include <esp_gatts_api.h>
#include <esp_gap_ble_api.h>
#include "esphome/components/esp32_ble_server/ble_server.h"
#endif

namespace esphome {
namespace bus_sniffer {

class BusSniffer : public Component {
 public:
  void setup() override;
  void loop() override;
#if UNNI_BLE_ENABLED
  void configure_gatt_server(esp32_ble_server::BLEServer *server);

  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

  void gatts_event_handler(esp_gatts_cb_event_t event,
                           esp_gatt_if_t gatts_if,
                           esp_ble_gatts_cb_param_t *param);

  void set_ble_advertising_interval(uint32_t interval_ms);
#endif
  void set_ha_publish_interval(uint32_t interval_ms) { this->ha_publish_interval_ms_ = interval_ms; }

  void set_co2_sensor(sensor::Sensor *sensor) {
    this->co2_sensor_ = sensor;
  }

  void set_crc_errors_sensor(sensor::Sensor *sensor) {
    this->crc_errors_sensor_ = sensor;
  }

  void set_frame_errors_sensor(sensor::Sensor *sensor) {
    this->frame_errors_sensor_ = sensor;
  }

  void set_rt_temperature_sensor(sensor::Sensor *sensor) {
    this->rt_temperature_sensor_ = sensor;
  }

  void set_rh_humidity_sensor(sensor::Sensor *sensor) {
    this->rh_humidity_sensor_ = sensor;
  }

  void set_debug_metrics(bool enabled) { this->debug_metrics_ = enabled; }
  void set_thermal_transient_on_rate(float value) {
    this->thermal_transient_on_rate_c_per_min_ = value;
  }
  void set_thermal_transient_off_rate(float value) {
    this->thermal_transient_off_rate_c_per_min_ = value;
  }

  void set_ref_period_sensor(sensor::Sensor *sensor) { this->ref_period_sensor_ = sensor; }
  void set_rt_period_sensor(sensor::Sensor *sensor) { this->rt_period_sensor_ = sensor; }
  void set_rh_state_period_sensor(sensor::Sensor *sensor) { this->rh_state_period_sensor_ = sensor; }
  void set_rt_ratio_sensor(sensor::Sensor *sensor) { this->rt_ratio_sensor_ = sensor; }
  void set_rh_ratio_sensor(sensor::Sensor *sensor) { this->rh_ratio_sensor_ = sensor; }
  void set_rh_log_sensor(sensor::Sensor *sensor) { this->rh_log_sensor_ = sensor; }
  void set_measurement_quality_sensor(sensor::Sensor *sensor) { this->measurement_quality_sensor_ = sensor; }

  void set_thermal_transient_sensor(binary_sensor::BinarySensor *sensor) {
    this->thermal_transient_sensor_ = sensor;
  }
  void set_temperature_extrapolation_sensor(binary_sensor::BinarySensor *sensor) {
    this->temperature_extrapolation_sensor_ = sensor;
  }
  void set_humidity_extrapolation_sensor(binary_sensor::BinarySensor *sensor) {
    this->humidity_extrapolation_sensor_ = sensor;
  }
  void set_calibration_extrapolation_sensor(binary_sensor::BinarySensor *sensor) {
    this->calibration_extrapolation_sensor_ = sensor;
  }

 protected:
  sensor::Sensor *co2_sensor_{nullptr};
  sensor::Sensor *crc_errors_sensor_{nullptr};
  sensor::Sensor *frame_errors_sensor_{nullptr};
  sensor::Sensor *rt_temperature_sensor_{nullptr};
  sensor::Sensor *rh_humidity_sensor_{nullptr};

  sensor::Sensor *ref_period_sensor_{nullptr};
  sensor::Sensor *rt_period_sensor_{nullptr};
  sensor::Sensor *rh_state_period_sensor_{nullptr};
  sensor::Sensor *rt_ratio_sensor_{nullptr};
  sensor::Sensor *rh_ratio_sensor_{nullptr};
  sensor::Sensor *rh_log_sensor_{nullptr};
  sensor::Sensor *measurement_quality_sensor_{nullptr};
  binary_sensor::BinarySensor *thermal_transient_sensor_{nullptr};
  binary_sensor::BinarySensor *temperature_extrapolation_sensor_{nullptr};
  binary_sensor::BinarySensor *humidity_extrapolation_sensor_{nullptr};
  binary_sensor::BinarySensor *calibration_extrapolation_sensor_{nullptr};

  void maybe_publish_ha_();

  uint32_t ha_publish_interval_ms_{30000};
  uint32_t last_ha_publish_ms_{0};
  bool ha_have_co2_{false};
  bool ha_have_temperature_{false};
  bool ha_have_humidity_{false};
  bool ha_initial_co2_published_{false};
  bool ha_initial_temperature_published_{false};
  bool ha_initial_humidity_published_{false};
  float ha_co2_{0.0f};
  float ha_temperature_{0.0f};
  float ha_humidity_{0.0f};

  bool debug_metrics_{false};
  float thermal_transient_on_rate_c_per_min_{0.8f};
  float thermal_transient_off_rate_c_per_min_{0.3f};
  bool thermal_transient_active_{false};
  bool have_last_valid_temperature_{false};
  float last_valid_temperature_c_{0.0f};
  uint32_t last_valid_temperature_ms_{0};

  bool have_last_ppm_{false};
  uint16_t last_ppm_{0};

  uint32_t crc_errors_{0};
  uint32_t frame_errors_{0};
};

}  // namespace bus_sniffer
}  // namespace esphome
