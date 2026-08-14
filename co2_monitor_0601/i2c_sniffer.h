// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "esphome/core/defines.h"

#include <cstdint>

#ifndef RTRH_DEBUG_CAPTURE
#define RTRH_DEBUG_CAPTURE 0
#endif

namespace esphome {
namespace co2_monitor_0601 {
namespace i2c_sniffer {

static constexpr uint8_t MAX_DATA_BYTES = 32;
static constexpr uint8_t MAX_FRAMES = 32;

enum class Direction : uint8_t {
  Write = 0,
  Read = 1,
};

enum class EndCondition : uint8_t {
  Stop = 0,
  RepeatedStart,
  CaptureEnd,
};

// One passively observed 7-bit I2C frame/segment. ACK entries describe the
// receiver response after the corresponding address/data byte.
struct Frame {
  uint8_t address{0};
  Direction direction{Direction::Write};
  bool address_ack{false};
  uint8_t data[MAX_DATA_BYTES]{};
  bool ack[MAX_DATA_BYTES]{};
  uint8_t length{0};
  bool truncated{false};
  EndCondition end_condition{EndCondition::CaptureEnd};
};

struct Capture {
  Frame frames[MAX_FRAMES]{};
  uint8_t frame_count{0};

  // Generic framing/capture failures. Consumers can fold these into their own
  // diagnostics without the I2C layer knowing anything about the protocol.
  uint32_t frame_errors{0};
};

// Passive GPIO capture. The sniffer never enables pulls and never drives SDA/SCL.
bool setup(uint8_t sda_pin, uint8_t scl_pin);

// Enable/disable edge capture. Enabling starts with a clean decoder state;
// disabling drops any partial frame collected so far.
void set_capture_enabled(bool enabled);

// Returns true when one quiet-period-delimited edge capture was consumed.
// A capture may contain multiple frames (for example across repeated STARTs).
bool poll(Capture &capture);

#if RTRH_DEBUG_CAPTURE
// Registers the raw logic-analyzer download endpoint (/capture).
void register_debug_handler();

// Compact log representation intended for frames not claimed by a protocol
// consumer. Called only from normal task context.
void log_frame(const Frame &frame, const char *label = "I2C frame");
#endif

}  // namespace i2c_sniffer
}  // namespace co2_monitor_0601
}  // namespace esphome
