// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#ifdef USE_HOST

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

#include <cstdint>
#include <string>

namespace esphome {
namespace co2_monitor_0601 {

class CO2Monitor0601;

class EnergySaveModeSwitch : public switch_::Switch {
 public:
  void set_parent(CO2Monitor0601 *parent) { this->parent_ = parent; }
 protected:
  void write_state(bool state) override;
  CO2Monitor0601 *parent_{nullptr};
};

class BlePairingModeSwitch : public switch_::Switch {
 public:
  void set_parent(CO2Monitor0601 *parent) { this->parent_ = parent; }
 protected:
  void write_state(bool state) override;
  CO2Monitor0601 *parent_{nullptr};
};

class WifiHaSwitch : public switch_::Switch {
 public:
  void set_parent(CO2Monitor0601 *parent) { this->parent_ = parent; }
 protected:
  void write_state(bool state) override;
  CO2Monitor0601 *parent_{nullptr};
};

// Native-host shim for ESPHome configuration/integration tests. It deliberately
// does not emulate GPIO, interrupts, PM locks, ADC, BLE radio/GATT or flash.
// Instead it keeps the public codegen surface of the production component and
// executes deterministic tests for the portable CO2 decoder and calibration.
class CO2Monitor0601 : public Component {
 public:
  void setup() override;
  void loop() override {}
  void prepare_for_ota() {}

  void set_ble_advertising_interval(uint32_t value) { this->ble_advertising_interval_ms_ = value; }
  void set_ble_battery_advertising_interval(uint32_t value) { this->ble_battery_advertising_interval_ms_ = value; }
  void set_ble_device_name(const std::string &value) { this->ble_device_name_ = value; }

  void set_ha_publish_interval(uint32_t value) { this->ha_publish_interval_ms_ = value; }
  void set_sniffer_enabled(bool value) { this->sniffer_enabled_ = value; }
  void set_rtrh_enabled(bool value) { this->rtrh_enabled_ = value; }
  void set_rtrh_edge_capture(bool value) { this->rtrh_edge_capture_ = value; }
  void set_rtrh_decode_only(bool value) { this->rtrh_decode_only_ = value; }
  void set_rtrh_gpio_setup(bool value) { this->rtrh_gpio_setup_ = value; }
  void set_sniffer_start_delay(uint32_t value) { this->sniffer_start_delay_ms_ = value; }
  void set_debug_metrics(bool value) { this->debug_metrics_ = value; }
  void set_active_i2c_probe(bool value) { this->active_i2c_probe_ = value; }
  void set_active_i2c_probe_interval(uint32_t value) { this->active_i2c_probe_interval_ms_ = value; }
  void set_debug_udp_host(const std::string &value) { this->debug_udp_host_ = value; }
  void set_debug_udp_port(uint16_t value) { this->debug_udp_port_ = value; }
  void set_light_sleep(bool value) { this->light_sleep_ = value; }
  void set_light_sleep_max_awake(uint32_t value) { this->light_sleep_max_awake_ms_ = value; }
  void set_co2_wake_idle_stable(uint32_t value) { this->co2_wake_idle_stable_ms_ = value; }
  void set_co2_wake_guard_time(uint32_t value) { this->co2_wake_guard_time_ms_ = value; }
  void set_rtrh_pins(uint8_t rt, uint8_t rh) { this->rt_pin_ = rt; this->rh_pin_ = rh; }
  void set_co2_pins(uint8_t sda, uint8_t scl) { this->co2_sda_pin_ = sda; this->co2_scl_pin_ = scl; }
  void set_battery_pin(uint8_t value) { this->battery_pin_ = value; }
  void set_battery_update_interval(uint32_t value) { this->battery_update_interval_ms_ = value; }
  void set_battery_divider_ratio(float value) { this->battery_divider_ratio_ = value; }
  void set_battery_learning_save_interval(uint32_t value) { this->battery_learning_save_interval_ms_ = value; }
  void set_usb_power_pin(uint8_t value) { this->usb_power_pin_ = value; }
  void set_energy_save_mode_default(bool value) { this->energy_save_mode_ = value; }
  void set_energy_save_grace(uint32_t value) { this->energy_save_grace_ms_ = value; }
  void set_wifi_recovery_window(uint32_t value) { this->wifi_recovery_window_ms_ = value; }
  void set_ble_pairing_window(uint32_t value) { this->ble_pairing_window_ms_ = value; }
  void set_thermal_transient_on_rate(float value) { this->thermal_transient_on_rate_ = value; }
  void set_thermal_transient_off_rate(float value) { this->thermal_transient_off_rate_ = value; }

  void set_energy_save_mode_switch(EnergySaveModeSwitch *value) { this->energy_save_switch_ = value; }
  void set_ble_pairing_mode_switch(BlePairingModeSwitch *value) { this->ble_pairing_switch_ = value; }
  void set_wifi_ha_switch(WifiHaSwitch *value) { this->wifi_ha_switch_ = value; }
  void set_energy_save_mode(bool enabled) { this->energy_save_mode_ = enabled; }
  void set_ble_pairing_mode(bool enabled) { this->ble_pairing_mode_ = enabled; }
  void set_wifi_ha_enabled(bool enabled) { this->wifi_ha_enabled_ = enabled; }

#define UNNI_HOST_SENSOR_SETTER(method, member) \
  void method(sensor::Sensor *value) { this->member = value; }
  UNNI_HOST_SENSOR_SETTER(set_co2_sensor, co2_)
  UNNI_HOST_SENSOR_SETTER(set_crc_errors_sensor, crc_errors_)
  UNNI_HOST_SENSOR_SETTER(set_frame_errors_sensor, frame_errors_)
  UNNI_HOST_SENSOR_SETTER(set_rt_temperature_sensor, rt_temperature_)
  UNNI_HOST_SENSOR_SETTER(set_air_temperature_sensor, air_temperature_)
  UNNI_HOST_SENSOR_SETTER(set_unni_display_temperature_sensor, display_temperature_)
  UNNI_HOST_SENSOR_SETTER(set_rh_humidity_sensor, rh_humidity_)
  UNNI_HOST_SENSOR_SETTER(set_unni_display_humidity_sensor, display_humidity_)
  UNNI_HOST_SENSOR_SETTER(set_battery_voltage_sensor, battery_voltage_)
  UNNI_HOST_SENSOR_SETTER(set_battery_level_sensor, battery_level_)
  UNNI_HOST_SENSOR_SETTER(set_battery_runtime_estimate_sensor, battery_runtime_estimate_)
  UNNI_HOST_SENSOR_SETTER(set_battery_charge_time_estimate_sensor, battery_charge_time_estimate_)
  UNNI_HOST_SENSOR_SETTER(set_battery_discharge_rate_sensor, battery_discharge_rate_)
  UNNI_HOST_SENSOR_SETTER(set_battery_charge_rate_sensor, battery_charge_rate_)
  UNNI_HOST_SENSOR_SETTER(set_battery_learned_full_runtime_sensor, battery_learned_full_runtime_)
  UNNI_HOST_SENSOR_SETTER(set_battery_learning_progress_sensor, battery_learning_progress_)
  UNNI_HOST_SENSOR_SETTER(set_battery_learning_cycles_sensor, battery_learning_cycles_)
  UNNI_HOST_SENSOR_SETTER(set_ref_period_sensor, ref_period_)
  UNNI_HOST_SENSOR_SETTER(set_rt_period_sensor, rt_period_)
  UNNI_HOST_SENSOR_SETTER(set_rh_state_period_sensor, rh_state_period_)
  UNNI_HOST_SENSOR_SETTER(set_rt_ratio_sensor, rt_ratio_)
  UNNI_HOST_SENSOR_SETTER(set_rh_ratio_sensor, rh_ratio_)
  UNNI_HOST_SENSOR_SETTER(set_rh_log_sensor, rh_log_)
  UNNI_HOST_SENSOR_SETTER(set_measurement_quality_sensor, measurement_quality_)
#undef UNNI_HOST_SENSOR_SETTER

#define UNNI_HOST_BINARY_SETTER(method, member) \
  void method(binary_sensor::BinarySensor *value) { this->member = value; }
  UNNI_HOST_BINARY_SETTER(set_usb_power_sensor, usb_power_)
  UNNI_HOST_BINARY_SETTER(set_thermal_transient_sensor, thermal_transient_)
  UNNI_HOST_BINARY_SETTER(set_temperature_extrapolation_sensor, temperature_extrapolation_)
  UNNI_HOST_BINARY_SETTER(set_humidity_extrapolation_sensor, humidity_extrapolation_)
  UNNI_HOST_BINARY_SETTER(set_calibration_extrapolation_sensor, calibration_extrapolation_)
#undef UNNI_HOST_BINARY_SETTER

 protected:
  bool run_portable_self_test_();
  bool run_capture_regression_tests_();
  bool run_current_capture_regression_tests_();
  void publish_fixture_values_();

  sensor::Sensor *co2_{nullptr};
  sensor::Sensor *crc_errors_{nullptr};
  sensor::Sensor *frame_errors_{nullptr};
  sensor::Sensor *rt_temperature_{nullptr};
  sensor::Sensor *air_temperature_{nullptr};
  sensor::Sensor *display_temperature_{nullptr};
  sensor::Sensor *rh_humidity_{nullptr};
  sensor::Sensor *display_humidity_{nullptr};
  sensor::Sensor *battery_voltage_{nullptr};
  sensor::Sensor *battery_level_{nullptr};
  sensor::Sensor *battery_runtime_estimate_{nullptr};
  sensor::Sensor *battery_charge_time_estimate_{nullptr};
  sensor::Sensor *battery_discharge_rate_{nullptr};
  sensor::Sensor *battery_charge_rate_{nullptr};
  sensor::Sensor *battery_learned_full_runtime_{nullptr};
  sensor::Sensor *battery_learning_progress_{nullptr};
  sensor::Sensor *battery_learning_cycles_{nullptr};
  sensor::Sensor *ref_period_{nullptr};
  sensor::Sensor *rt_period_{nullptr};
  sensor::Sensor *rh_state_period_{nullptr};
  sensor::Sensor *rt_ratio_{nullptr};
  sensor::Sensor *rh_ratio_{nullptr};
  sensor::Sensor *rh_log_{nullptr};
  sensor::Sensor *measurement_quality_{nullptr};
  binary_sensor::BinarySensor *usb_power_{nullptr};
  binary_sensor::BinarySensor *thermal_transient_{nullptr};
  binary_sensor::BinarySensor *temperature_extrapolation_{nullptr};
  binary_sensor::BinarySensor *humidity_extrapolation_{nullptr};
  binary_sensor::BinarySensor *calibration_extrapolation_{nullptr};

  EnergySaveModeSwitch *energy_save_switch_{nullptr};
  BlePairingModeSwitch *ble_pairing_switch_{nullptr};
  WifiHaSwitch *wifi_ha_switch_{nullptr};

  std::string ble_device_name_{};
  std::string debug_udp_host_{};
  uint32_t ble_advertising_interval_ms_{0};
  uint32_t ble_battery_advertising_interval_ms_{0};
  uint32_t ha_publish_interval_ms_{0};
  uint32_t sniffer_start_delay_ms_{0};
  uint32_t active_i2c_probe_interval_ms_{0};
  uint32_t light_sleep_max_awake_ms_{0};
  uint32_t co2_wake_idle_stable_ms_{0};
  uint32_t co2_wake_guard_time_ms_{0};
  uint32_t battery_update_interval_ms_{0};
  uint32_t battery_learning_save_interval_ms_{0};
  uint32_t energy_save_grace_ms_{0};
  uint32_t wifi_recovery_window_ms_{0};
  uint32_t ble_pairing_window_ms_{0};
  uint16_t debug_udp_port_{0};
  uint8_t rt_pin_{0};
  uint8_t rh_pin_{0};
  uint8_t co2_sda_pin_{0};
  uint8_t co2_scl_pin_{0};
  uint8_t battery_pin_{0};
  uint8_t usb_power_pin_{0};
  float battery_divider_ratio_{0.0f};
  float thermal_transient_on_rate_{0.0f};
  float thermal_transient_off_rate_{0.0f};
  bool sniffer_enabled_{false};
  bool rtrh_enabled_{false};
  bool rtrh_edge_capture_{false};
  bool rtrh_decode_only_{false};
  bool rtrh_gpio_setup_{false};
  bool debug_metrics_{false};
  bool active_i2c_probe_{false};
  bool light_sleep_{false};
  bool energy_save_mode_{false};
  bool ble_pairing_mode_{false};
  bool wifi_ha_enabled_{true};
};

}  // namespace co2_monitor_0601
}  // namespace esphome

#endif  // USE_HOST
