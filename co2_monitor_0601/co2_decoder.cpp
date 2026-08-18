// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "co2_decoder.h"

namespace esphome {
namespace co2_monitor_0601 {
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

bool validate_measurement_capture(const i2c_sniffer::Capture &capture) {
  if (capture.frame_errors != 0 || capture.frame_count != 2) return false;

  const auto &command = capture.frames[0];
  const auto &response = capture.frames[1];

  if (!i2c_sniffer::frame_valid(command) ||
      command.address != CO2_I2C_ADDRESS ||
      command.direction != i2c_sniffer::Direction::Write ||
      !command.address_ack || command.length != 2 ||
      command.data[0] != 0xEC || command.data[1] != 0x05 ||
      !command.ack[0] || !command.ack[1] ||
      command.end_condition != i2c_sniffer::EndCondition::Stop)
    return false;

  if (!i2c_sniffer::frame_valid(response) ||
      response.address != CO2_I2C_ADDRESS ||
      response.direction != i2c_sniffer::Direction::Read ||
      !response.address_ack || response.length != 3 ||
      !response.ack[0] || !response.ack[1] || response.ack[2] ||
      response.end_condition != i2c_sniffer::EndCondition::Stop)
    return false;

  return response.data[2] == sensirion_crc(response.data[0], response.data[1]);
}

bool process_frame(const i2c_sniffer::Frame &frame, Result &result) {
  // Structural capture/framing failures belong to the generic I2C layer and
  // must never be interpreted as protocol data.
  if (!i2c_sniffer::frame_valid(frame)) return false;
  if (frame.address != CO2_I2C_ADDRESS) return false;

  if (frame.direction == i2c_sniffer::Direction::Write) {
    // Observed SCD4x-compatible read_measurement command: 0xEC05.
    if (frame.length >= 2 && frame.data[0] == 0xEC && frame.data[1] == 0x05)
      return true;
    return false;
  }

  // Preserve the previous decoder behavior: every read from the observed CO2
  // address is treated as the measurement response candidate.
  if (frame.length != 3 || frame.end_condition != i2c_sniffer::EndCondition::Stop) {
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


bool process_capture(const i2c_sniffer::Capture &capture, Result &result) {
  bool measurement_armed = false;
  result.frame_errors += capture.frame_errors;

  for (uint8_t i = 0; i < capture.frame_count; i++) {
    const auto &frame = capture.frames[i];
    if (!i2c_sniffer::frame_valid(frame)) {
      // poll() already accounts structural failures in capture.frame_errors.
      measurement_armed = false;
      continue;
    }
    if (frame.address != CO2_I2C_ADDRESS) continue;

    if (frame.direction == i2c_sniffer::Direction::Write) {
      const bool valid_command =
          frame.address_ack && frame.length == 2 &&
          frame.data[0] == 0xEC && frame.data[1] == 0x05 &&
          frame.ack[0] && frame.ack[1] &&
          frame.end_condition == i2c_sniffer::EndCondition::Stop;
      if (valid_command) {
        measurement_armed = true;
      } else {
        // Only malformed frames that look like the observed measurement
        // command are protocol errors. Other writes to 0x62 are unrelated.
        if (frame.length >= 1 && frame.data[0] == 0xEC) result.frame_errors++;
        measurement_armed = false;
      }
      continue;
    }

    // Never accept an arbitrary CRC-valid read from 0x62 as a CO2 sample.
    if (!measurement_armed) {
      result.frame_errors++;
      continue;
    }
    measurement_armed = false;
    process_frame(frame, result);
  }

  return result.have_co2;
}

}  // namespace co2_decoder
}  // namespace co2_monitor_0601
}  // namespace esphome
