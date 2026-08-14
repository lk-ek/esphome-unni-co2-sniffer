// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "ble_options.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#if UNNI_BLE_ENABLED
#include "esphome/components/esp32_ble_server/ble_server.h"
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#endif

namespace esphome {
namespace bus_sniffer {

class BusSniffer : public Component {
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

  void maybe_publish_ha_();
  void publish_cached_ha_now_();
  bool usb_powered_() const { return this->usb_power_.have_state && this->usb_power_.state; }
  void process_rtrh_();
  void process_co2_();
  bool setup_battery_adc_();
  void process_battery_();
  bool setup_usb_power_();
  void process_usb_power_();
  static float battery_percent_from_voltage_(float voltage);
  float update_thermal_transient_(float temperature_c);
  bool initialize_sniffer_io_();

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
  uint32_t light_sleep_max_awake_ms_{10000};
  uint8_t rt_pin_{3};
  uint8_t rh_pin_{4};
  uint8_t co2_sda_pin_{6};
  uint8_t co2_scl_pin_{7};
};

}  // namespace bus_sniffer
}  // namespace esphome
