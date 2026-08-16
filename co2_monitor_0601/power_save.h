// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include "esp_attr.h"

namespace esphome {
namespace co2_monitor_0601 {
namespace power_save {

// Configure ESP-IDF automatic Light-sleep. The RT/RH GPIOs are wake sources;
// the CO2 bus GPIOs deliberately are not.
bool setup(bool enabled, uint32_t max_awake_ms, uint8_t rt_pin, uint8_t rh_pin,
           uint8_t co2_sda_pin, uint8_t co2_scl_pin);

// Called from the RT/RH GPIO ISR. esp_pm_lock_acquire() is explicitly ISR-safe.
void on_rtrh_edge_from_isr();

// Monotonic generation incremented when an RT/RH edge opens a new battery
// awake window after automatic Light-sleep. Task code can use this to restore
// passive GPIO state outside ISR context.
uint32_t wake_generation();

// Called once the corresponding decoders have produced complete data.
void on_rtrh_complete(bool valid);
void on_valid_co2();

// Keep automatic Light-sleep inhibited until queued debug-network traffic has
// drained, with a bounded grace period enforced by loop().
void set_transport_busy(bool busy);

// Keep the ESP fully awake at 80 MHz while external USB/VBUS power is present.
// Battery mode releases these persistent locks and falls back to the RT/RH wake window.
void set_external_power(bool present);
bool external_power_present();

// Releases the no-Light-sleep lock when the wake cycle is complete or a
// failsafe timeout expires.
void loop();

bool enabled();
bool awake_window_active();

}  // namespace power_save
}  // namespace co2_monitor_0601
}  // namespace esphome
