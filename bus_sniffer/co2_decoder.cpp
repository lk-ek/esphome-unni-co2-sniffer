// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "co2_decoder.h"

namespace esphome {
namespace bus_sniffer {
namespace co2_decoder {

static constexpr uint8_t CO2_I2C_ADDRESS = 0x62;

// Protocol provenance: this CRC uses the Sensirion/SCD4x-compatible wire
// parameters observed by the passive decoder (initial value 0xFF, polynomial
// 0x31). It is local protocol code, not a vendored Sensirion driver.
static uint8_t sensirion_crc(uint8_t byte0, uint8_t byte1) {
  uint8_t crc = 0xff;
  const uint8_t data[2] = {byte0, byte1};
  for (uint8_t value : data) {
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                         : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

bool process_frame(const i2c_sniffer::Frame &frame, Result &result) {
  if (frame.address != CO2_I2C_ADDRESS) return false;

  if (frame.direction == i2c_sniffer::Direction::Write) {
    // Observed SCD4x-compatible read_measurement command: 0xEC05.
    if (frame.length >= 2 && frame.data[0] == 0xEC && frame.data[1] == 0x05)
      return true;
    return false;
  }

  // Preserve the previous decoder behavior: every read from the observed CO2
  // address is treated as the measurement response candidate.
  if (frame.length < 3) {
    result.frame_errors++;
    return true;
  }
  if (!frame.address_ack || !frame.ack[0] || !frame.ack[1] || frame.ack[2]) {
    result.frame_errors++;
    return true;
  }

  const uint8_t msb = frame.data[0];
  const uint8_t lsb = frame.data[1];
  if (frame.data[2] != sensirion_crc(msb, lsb)) {
    result.crc_errors++;
    return true;
  }

  result.co2_ppm = (static_cast<uint16_t>(msb) << 8) | lsb;
  result.have_co2 = true;
  return true;
}

}  // namespace co2_decoder
}  // namespace bus_sniffer
}  // namespace esphome
