// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include "esp_attr.h"

namespace esphome {
namespace bus_sniffer {
namespace power_save {

// Configure ESP-IDF automatic Light-sleep. GPIO3/GPIO4 are used only as
// wake sources; CO2 GPIO6/GPIO7 deliberately are not.
bool setup(bool enabled, uint32_t max_awake_ms);

// Called from the RT/RH GPIO ISR. esp_pm_lock_acquire() is explicitly ISR-safe.
void on_rtrh_edge_from_isr();

// Called once the corresponding decoders have produced complete data.
void on_rtrh_complete(bool valid);
void on_valid_co2();

// Releases the no-Light-sleep lock when the wake cycle is complete or a
// failsafe timeout expires.
void loop();

bool enabled();
bool awake_window_active();

}  // namespace power_save
}  // namespace bus_sniffer
}  // namespace esphome
