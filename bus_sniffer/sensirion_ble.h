#pragma once
#include "ble_options.h"
#if UNNI_BLE_ENABLED

#include <cstdint>
#include <esp_gap_ble_api.h>

namespace esphome {
namespace bus_sniffer {

void sensirion_ble_setup();
void sensirion_ble_set_advertising_interval(uint32_t interval_ms);
void sensirion_ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void sensirion_ble_set_temperature_humidity(float temperature_c, float humidity_percent);
void sensirion_ble_set_co2(uint16_t ppm);

uint16_t sensirion_ble_encode_temperature(float value);
uint16_t sensirion_ble_encode_humidity(float value);
uint16_t sensirion_ble_get_device_id();

bool sensirion_ble_has_temperature();
bool sensirion_ble_has_humidity();
bool sensirion_ble_has_co2();
float sensirion_ble_temperature();
float sensirion_ble_humidity();
uint16_t sensirion_ble_co2();

}  // namespace bus_sniffer
}  // namespace esphome

#endif  // UNNI_BLE_ENABLED
