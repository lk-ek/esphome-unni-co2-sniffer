// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "ble_options.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#if UNNI_BLE_ENABLED
#include "esphome/components/esp32_ble_server/ble_server.h"
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#endif

namespace esphome {
namespace co2_monitor_0601 {

class CO2Monitor0601;

#if UNNI_BLE_ENABLED
class BlePairingModeSwitch : public switch_::Switch {
 public:
  void set_parent(CO2Monitor0601 *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;
  CO2Monitor0601 *parent_{nullptr};
};
#endif

class EnergySaveModeSwitch : public switch_::Switch {
 public:
  void set_parent(CO2Monitor0601 *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;
  CO2Monitor0601 *parent_{nullptr};
};

class CO2Monitor0601 : public Component {
 public:
  void setup() override;
  void loop() override;

#if UNNI_BLE_ENABLED
  void set_gatt_server(esp32_ble_server::BLEServer *server) { this->gatt_server_ = server; }
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                           esp_ble_gatts_cb_param_t *param);
  void set_ble_advertising_interval(uint32_t interval_ms);
  void set_ble_battery_advertising_interval(uint32_t interval_ms) { this->ble_battery_advertising_interval_ms_ = interval_ms; }
#endif

  void set_ha_publish_interval(uint32_t value) { this->ha_.interval_ms = value; }
  void set_sniffer_enabled(bool value) { this->sniffer_enabled_ = value; }
  void set_rtrh_enabled(bool value) { this->rtrh_enabled_ = value; }
  void set_rtrh_edge_capture(bool value) { this->rtrh_edge_capture_ = value; }
  void set_rtrh_decode_only(bool value) { this->rtrh_decode_only_ = value; }
  void set_rtrh_gpio_setup(bool value) { this->rtrh_gpio_setup_ = value; }
  void set_sniffer_start_delay(uint32_t value) { this->start_delay_ms_ = value; }
  void set_debug_metrics(bool value) { this->debug_metrics_ = value; }
  void set_light_sleep(bool value) { this->light_sleep_enabled_ = value; }
  void set_light_sleep_max_awake(uint32_t value) { this->light_sleep_max_awake_ms_ = value; }
  void set_rtrh_pins(uint8_t rt, uint8_t rh) { this->rt_pin_ = rt; this->rh_pin_ = rh; }
  void set_co2_pins(uint8_t sda, uint8_t scl) { this->co2_sda_pin_ = sda; this->co2_scl_pin_ = scl; }
  void set_battery_pin(uint8_t pin) { this->battery_.pin = pin; }
  void set_battery_update_interval(uint32_t value) { this->battery_.interval_ms = value; }
  void set_battery_divider_ratio(float value) { this->battery_.divider_ratio = value; }
  void set_usb_power_pin(uint8_t pin) { this->usb_power_.pin = pin; }
  void set_energy_save_mode_default(bool value) {
    this->energy_save_mode_ = value;
    this->energy_save_policy_active_ = value;
  }
  void set_energy_save_grace(uint32_t value) { this->energy_save_grace_ms_ = value; }
  void set_energy_save_mode_switch(EnergySaveModeSwitch *s) { this->energy_save_switch_ = s; }
#if UNNI_BLE_ENABLED
  void set_ble_pairing_mode_switch(BlePairingModeSwitch *s) { this->ble_pairing_switch_ = s; }
  void set_ble_pairing_window(uint32_t value) { this->ble_pairing_window_ms_ = value; }
  void set_ble_pairing_mode(bool enabled);
#endif
  void set_energy_save_mode(bool enabled);
  void set_thermal_transient_on_rate(float value) { this->thermal_.on_rate = value; }
  void set_thermal_transient_off_rate(float value) { this->thermal_.off_rate = value; }

  // ESPHome codegen setters. Keeping these explicit preserves the existing YAML
  // API while all runtime state is grouped below by responsibility.
  void set_co2_sensor(sensor::Sensor *s) { this->out_.co2 = s; }
  void set_crc_errors_sensor(sensor::Sensor *s) { this->out_.crc_errors = s; }
  void set_frame_errors_sensor(sensor::Sensor *s) { this->out_.frame_errors = s; }
  void set_rt_temperature_sensor(sensor::Sensor *s) { this->out_.temperature = s; }
  void set_rh_humidity_sensor(sensor::Sensor *s) { this->out_.humidity = s; }
  void set_battery_voltage_sensor(sensor::Sensor *s) { this->out_.battery_voltage = s; }
  void set_battery_level_sensor(sensor::Sensor *s) { this->out_.battery_level = s; }
  void set_usb_power_sensor(binary_sensor::BinarySensor *s) { this->out_.usb_power = s; }
  void set_ref_period_sensor(sensor::Sensor *s) { this->out_.ref_period = s; }
  void set_rt_period_sensor(sensor::Sensor *s) { this->out_.rt_period = s; }
  void set_rh_state_period_sensor(sensor::Sensor *s) { this->out_.rh_state_period = s; }
  void set_rt_ratio_sensor(sensor::Sensor *s) { this->out_.rt_ratio = s; }
  void set_rh_ratio_sensor(sensor::Sensor *s) { this->out_.rh_ratio = s; }
  void set_rh_log_sensor(sensor::Sensor *s) { this->out_.rh_log = s; }
  void set_measurement_quality_sensor(sensor::Sensor *s) { this->out_.quality = s; }
  void set_thermal_transient_sensor(binary_sensor::BinarySensor *s) { this->out_.thermal_transient = s; }
  void set_temperature_extrapolation_sensor(binary_sensor::BinarySensor *s) { this->out_.temperature_extrapolation = s; }
  void set_humidity_extrapolation_sensor(binary_sensor::BinarySensor *s) { this->out_.humidity_extrapolation = s; }
  void set_calibration_extrapolation_sensor(binary_sensor::BinarySensor *s) { this->out_.calibration_extrapolation = s; }

 protected:
  struct Outputs {
    sensor::Sensor *co2{nullptr};
    sensor::Sensor *crc_errors{nullptr};
    sensor::Sensor *frame_errors{nullptr};
    sensor::Sensor *temperature{nullptr};
    sensor::Sensor *humidity{nullptr};
    sensor::Sensor *battery_voltage{nullptr};
    sensor::Sensor *battery_level{nullptr};
    binary_sensor::BinarySensor *usb_power{nullptr};
    sensor::Sensor *ref_period{nullptr};
    sensor::Sensor *rt_period{nullptr};
    sensor::Sensor *rh_state_period{nullptr};
    sensor::Sensor *rt_ratio{nullptr};
    sensor::Sensor *rh_ratio{nullptr};
    sensor::Sensor *rh_log{nullptr};
    sensor::Sensor *quality{nullptr};
    binary_sensor::BinarySensor *thermal_transient{nullptr};
    binary_sensor::BinarySensor *temperature_extrapolation{nullptr};
    binary_sensor::BinarySensor *humidity_extrapolation{nullptr};
    binary_sensor::BinarySensor *calibration_extrapolation{nullptr};
  } out_;

  struct HaState {
    uint32_t interval_ms{60000};
    uint32_t last_publish_ms{0};
    bool have_co2{false};
    bool have_temperature{false};
    bool have_humidity{false};
    bool initial_co2_published{false};
    bool initial_temperature_published{false};
    bool initial_humidity_published{false};
    float co2{0.0f};
    float temperature{0.0f};
    float humidity{0.0f};
  } ha_;

  struct ThermalState {
    float on_rate{0.8f};
    float off_rate{0.3f};
    bool active{false};
    bool have_previous{false};
    float previous_temperature{0.0f};
    uint32_t previous_ms{0};
  } thermal_;

  struct Co2State {
    bool have_last_ppm{false};
    uint16_t last_ppm{0};
    uint32_t crc_errors{0};
    uint32_t frame_errors{0};
  } co2_;

  struct BatteryState {
    uint8_t pin{2};
    uint32_t interval_ms{60000};
    uint32_t last_measure_ms{0};
    float divider_ratio{2.0f};
    bool initialized{false};
    adc_unit_t unit{ADC_UNIT_1};
    adc_channel_t channel{ADC_CHANNEL_2};
    adc_oneshot_unit_handle_t adc_handle{nullptr};
    adc_cali_handle_t cali_handle{nullptr};
  } battery_;

  struct UsbPowerState {
    uint8_t pin{5};
    bool initialized{false};
    bool have_state{false};
    bool state{false};
    bool candidate{false};
    uint32_t candidate_since_ms{0};
  } usb_power_;

  // Temporary runtime instrumentation for the intermittent Native API stall.
  // All timing is collected from the normal component loop; no ISR path is
  // modified by this diagnostic. Maxima are reported/reset once per second.
  struct RuntimeDiagState {
    uint64_t last_loop_start_us{0};
    uint64_t current_loop_start_us{0};
    uint32_t last_report_ms{0};
    uint32_t last_heap_report_ms{0};
    uint32_t loops{0};
    uint32_t max_loop_gap_us{0};
    uint32_t max_component_us{0};
    uint32_t max_history_us{0};
    uint32_t max_policy_us{0};
    uint32_t max_ha_publish_us{0};
    uint32_t max_battery_us{0};
    uint32_t max_rtrh_us{0};
    uint32_t max_co2_us{0};
    uint32_t max_power_save_us{0};
  } runtime_diag_;

  void maybe_publish_ha_();
  void publish_cached_ha_now_();
  bool usb_powered_() const { return this->usb_power_.have_state && this->usb_power_.state; }
  bool external_powered_() const { return this->usb_powered_() && !this->energy_save_policy_active_; }
  void process_energy_save_grace_();
#if UNNI_BLE_ENABLED
  void process_ble_pairing_window_();
  void begin_ble_security_(esp_bd_addr_t remote_bda);
#endif
  void apply_power_policy_(bool force = false);
  void process_rtrh_(bool publish_outputs = true);
  void process_co2_();
  bool setup_battery_adc_();
  void process_battery_();
  bool setup_usb_power_();
  void process_usb_power_();
  static float battery_percent_from_voltage_(float voltage);
  float update_thermal_transient_(float temperature_c);
  bool initialize_sniffer_io_();
  void runtime_diag_loop_begin_();
  void runtime_diag_loop_end_();
  static void runtime_diag_update_max_(uint32_t &slot, uint64_t elapsed_us);

  bool sniffer_enabled_{true};
  bool rtrh_enabled_{true};
  bool rtrh_edge_capture_{false};
  bool rtrh_decode_only_{false};
  bool rtrh_gpio_setup_{false};
  uint32_t start_delay_ms_{0};
  uint32_t boot_ms_{0};
  bool io_initialized_{false};
  bool debug_metrics_{false};
#if UNNI_BLE_ENABLED
  esp32_ble_server::BLEServer *gatt_server_{nullptr};
  uint32_t ble_usb_advertising_interval_ms_{2000};
  uint32_t ble_battery_advertising_interval_ms_{5000};
#endif
  bool light_sleep_enabled_{true};
  bool energy_save_mode_{false};
  bool energy_save_policy_active_{false};
  bool energy_save_grace_pending_{false};
  uint32_t energy_save_grace_started_ms_{0};
  uint32_t energy_save_grace_ms_{3000};
  bool power_policy_have_state_{false};
  bool power_policy_external_power_{false};
  EnergySaveModeSwitch *energy_save_switch_{nullptr};
#if UNNI_BLE_ENABLED
  BlePairingModeSwitch *ble_pairing_switch_{nullptr};
  bool ble_pairing_mode_{false};
  uint32_t ble_pairing_started_ms_{0};
  uint32_t ble_pairing_window_ms_{60000};
#endif
  uint32_t light_sleep_max_awake_ms_{10000};
  uint8_t rt_pin_{3};
  uint8_t rh_pin_{4};
  uint8_t co2_sda_pin_{6};
  uint8_t co2_scl_pin_{7};
};

}  // namespace co2_monitor_0601
}  // namespace esphome
