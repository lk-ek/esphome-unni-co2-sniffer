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

enum class FrameStatus : uint8_t {
  Valid = 0,
  IncompleteByte,
  CaptureEndedInFrame,
  Truncated,
};

// One passively observed 7-bit I2C frame/segment. ACK entries describe the
// receiver response after the corresponding address/data byte. Structural
// capture/framing failures are carried in status and must not be passed to a
// protocol decoder.
struct Frame {
  uint8_t address{0};
  Direction direction{Direction::Write};
  bool address_ack{false};
  uint8_t data[MAX_DATA_BYTES]{};
  bool ack[MAX_DATA_BYTES]{};
  uint8_t length{0};
  EndCondition end_condition{EndCondition::CaptureEnd};
  FrameStatus status{FrameStatus::Valid};
  // Number of bits already sampled for an unfinished byte (0 when aligned).
  uint8_t partial_bits{0};
};

inline bool frame_valid(const Frame &frame) {
  return frame.status == FrameStatus::Valid;
}

struct Capture {
  Frame frames[MAX_FRAMES]{};
  uint8_t frame_count{0};

  // Generic framing/capture failures. Consumers can fold these into their own
  // diagnostics without the I2C layer knowing anything about the protocol.
  uint32_t frame_errors{0};
#if RTRH_DEBUG_CAPTURE
  // Sequence of the raw /capture snapshot corresponding to this decoded
  // capture. Zero means it was not stored (for example because an earlier
  // suspicious capture is still frozen).
  uint32_t debug_raw_sequence{0};
#endif
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
// consumer or structurally malformed frames. Called only from normal task
// context.
void log_frame(const Frame &frame, const char *label = "I2C frame");

// Freeze the raw capture most recently returned by poll(). While frozen, later
// captures cannot overwrite /capture. The first successful GET of /capture
// releases the freeze again. Returns true when a new freeze was established.
bool freeze_last_capture(uint32_t sequence, const char *reason);
#endif

}  // namespace i2c_sniffer
}  // namespace co2_monitor_0601
}  // namespace esphome
