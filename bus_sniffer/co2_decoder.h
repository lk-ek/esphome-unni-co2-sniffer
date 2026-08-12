// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "esphome/core/defines.h"
#include <cstdint>

#ifndef RTRH_DEBUG_CAPTURE
#define RTRH_DEBUG_CAPTURE 0
#endif

namespace esphome {
namespace bus_sniffer {
namespace co2_decoder {

struct Result {
  bool have_co2{false};
  uint16_t co2_ppm{0};
  uint32_t crc_errors{0};
  uint32_t frame_errors{0};
};

// GPIO6/D4 = SDA, GPIO7/D5 = SCL.
bool setup();

// Enable/disable edge capture. Enabling starts with a clean decoder state;
// disabling drops any partial transaction collected so far.
void set_capture_enabled(bool enabled);

// Returns true when a complete capture was consumed. The result may contain
// only diagnostics (CRC/frame errors) and no CO2 value.
bool poll(Result &result);

#if RTRH_DEBUG_CAPTURE
void register_debug_handler();
#endif

}  // namespace co2_decoder
}  // namespace bus_sniffer
}  // namespace esphome
