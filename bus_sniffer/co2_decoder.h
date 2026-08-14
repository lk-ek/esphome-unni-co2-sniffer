// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "i2c_sniffer.h"

#include <cstdint>

namespace esphome {
namespace bus_sniffer {
namespace co2_decoder {

struct Result {
  bool have_co2{false};
  uint16_t co2_ppm{0};
  uint32_t crc_errors{0};
  uint32_t frame_errors{0};
};

// Consume one generic I2C frame. Returns true if the frame belongs to the
// observed CO2 protocol, even when it is malformed and only updates diagnostics.
bool process_frame(const i2c_sniffer::Frame &frame, Result &result);

}  // namespace co2_decoder
}  // namespace bus_sniffer
}  // namespace esphome
