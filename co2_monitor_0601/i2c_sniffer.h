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

struct Capture;
using CaptureValidator = bool (*)(const Capture &capture);

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

  // Raw GPIO SCL transitions in this capture, before any RMT/software repair.
  uint16_t raw_scl_edges{0};

  // Shared-GPIO ISR latency can collapse closely spaced SDA/SCL changes.
  // These diagnostics describe conservative decoder-side recovery only.
  uint16_t coalesced_edges_resolved{0};

  // Conservative software recovery: up to two complete SCL pulses may be
  // reconstructed from unusually long constant-SCL intervals, but only when
  // a caller-supplied protocol validator accepts one unique decoded result.
  // Timing alone can never make a repair valid.
  uint8_t recovered_missing_clocks{0};
  uint32_t recovered_gap_us[2]{};

  // Optional RMT-SCL assist can use the hardware-captured clock waveform to
  // restore SCL edges that were absent from the shared GPIO ISR stream.
  uint8_t rmt_scl_edges_recovered{0};
#if RTRH_DEBUG_CAPTURE
  // Sequence of the raw /capture snapshot corresponding to this decoded
  // capture. Zero means it was not stored (for example because an earlier
  // suspicious capture is still frozen).
  uint32_t debug_raw_sequence{0};
#endif
};

// Passive GPIO capture. The sniffer never enables pulls and never drives SDA/SCL.
bool setup(uint8_t sda_pin, uint8_t scl_pin, bool rmt_scl_assist = false);

// Remove any installed GPIO handlers/RMT resources after a partial or failed
// setup. Signal pins remain passive inputs without internal pulls.
void shutdown();

// Enable/disable edge capture. Enabling starts with a clean decoder state;
// disabling drops any partial frame collected so far.
void set_capture_enabled(bool enabled);

// Enable the one-shot CO2 warm-up activity handoff. While enabled, the first
// real SDA/SCL transition asks the power-save layer to acquire its ISR-only
// PM-lock pair. Disabled during normal capture so the hot ISR pays only one
// predictable branch and no cross-component call.
void set_co2_warmup_activity_watch(bool enabled);

// Task-context snapshot used to keep self-generated BLE history traffic away
// from a real edge capture. This is false while the enabled sniffer is idle.
bool capture_in_progress();

// Re-assert the passive GPIO input/interrupt configuration after automatic
// Light-sleep wake. This never enables pulls and never drives SDA/SCL.
void rearm_after_light_sleep();

// Experimental diagnostic helpers. These temporarily take ownership of the two
// tap GPIOs, disable the passive edge interrupts, perform a slow master
// transaction through the external 10 kOhm series resistors, and then restore
// the normal no-pull passive sniffer configuration. They are intended only for
// a bus that has been electrically quiet LOW/LOW for a long time.
bool bus_is_low_low();
uint32_t last_edge_age_us();
bool active_write_command(uint16_t command);
bool active_read_command(uint16_t command, uint8_t *data, uint8_t length);

// Returns true when one quiet-period-delimited edge capture was consumed.
// A capture may contain multiple frames (for example across repeated STARTs).
bool poll(Capture &capture, CaptureValidator recovery_validator = nullptr);

// Emit a 5-second summary of raw GPIO activity on the passive I2C tap.
void log_edge_diagnostics(uint32_t now_ms);

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

// True while a debug UDP capture is still being drained to the collector.
bool debug_export_pending();
#endif

}  // namespace i2c_sniffer
}  // namespace co2_monitor_0601
}  // namespace esphome
