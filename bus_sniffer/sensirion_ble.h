#pragma once

#include <cstdint>

namespace esphome {
namespace bus_sniffer {

void sensirion_ble_setup();
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
