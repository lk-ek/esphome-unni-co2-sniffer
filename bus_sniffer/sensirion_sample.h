// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Portions of the BLE sample encoding and T_RH_CO2_ALT byte layout are
// adapted from / compatible with Sensirion UPT Core 0.5.1 (BSD-3-Clause).
// Copyright 2024 Sensirion AG. See THIRD_PARTY_NOTICES.md and LICENSES/.
#pragma once

#include <array>
#include <cstdint>

namespace esphome {
namespace bus_sniffer {

struct SensirionSample {
  float temperature_c{0.0f};
  float humidity_percent{0.0f};
  uint16_t co2_ppm{0};
  bool have_temperature{false};
  bool have_humidity{false};
  bool have_co2{false};

  bool complete() const {
    return have_temperature && have_humidity && have_co2;
  }

  static uint16_t encode_temperature(float value) {
    // Sensirion UPT BLEProtocol::encodeTemperatureV1().
    return static_cast<uint16_t>((((value + 45.0f) / 175.0f) * 65535.0f) + 0.5f);
  }

  static uint16_t encode_humidity(float value) {
    // Sensirion UPT BLEProtocol::encodeHumidityV1().
    return static_cast<uint16_t>(((value / 100.0f) * 65535.0f) + 0.5f);
  }

  std::array<uint8_t, 8> encoded() const {
    std::array<uint8_t, 8> out{};
    put_u16_le(out.data() + 0, encode_temperature(temperature_c));
    put_u16_le(out.data() + 2, encode_humidity(humidity_percent));
    put_u16_le(out.data() + 4, co2_ppm);
    return out;
  }

  static void put_u16_le(uint8_t *p, uint16_t value) {
    p[0] = static_cast<uint8_t>(value & 0xFF);
    p[1] = static_cast<uint8_t>(value >> 8);
  }
};

}  // namespace bus_sniffer
}  // namespace esphome
