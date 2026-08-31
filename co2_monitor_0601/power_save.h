// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace esphome {
namespace co2_monitor_0601 {
namespace power_save {

// Configure ESP-IDF automatic Light-sleep. RT/RH remain wake sources.
// CO2 wake sources are armed dynamically: SCL HIGH while the Unni bus is
// powered down (quiet LOW/LOW), then SDA/SCL LOW during the powered high/high
// warm-up so the first native bus activity can promote capture to full power.
bool setup(bool enabled, uint32_t max_awake_ms, uint8_t rt_pin, uint8_t rh_pin,
           uint8_t co2_sda_pin, uint8_t co2_scl_pin);

// Called from the RT/RH GPIO ISR. esp_pm_lock_acquire() is explicitly ISR-safe.
void on_rtrh_edge_from_isr();

// Called from the passive I2C GPIO ISR. During the low-power CO2 warm-up this
// acquires a dedicated ISR-only PM-lock pair on the first real bus transition.
// Task context later hands ownership over to the normal CO2 lock pair.
void on_co2_edge_from_isr();

// Monotonic generation incremented when an RT/RH edge opens a new battery
// awake window after automatic Light-sleep. Task code can use this to restore
// passive GPIO state outside ISR context.
uint32_t wake_generation();

// Called once the RT/RH decoder has consumed a complete capture. Validation is
// separate: even a rejected capture closes the wake window so battery mode does
// not remain awake until the timeout.
void on_rtrh_complete();
void on_valid_co2();

// Track the Unni CO2 power window. A quiet LOW/LOW bus arms GPIO-high wake on
// SCL. After power-up, begin_co2_warmup() permits automatic Light Sleep and
// 40 MHz operation while SDA/SCL idle HIGH. The first observed bus transition
// acquires ISR-only handoff locks; promote_co2_warmup() transfers that hold to
// the normal task-owned CO2 locks for the active native capture window.
void set_co2_bus_powered_down(bool powered_down);
void begin_co2_warmup();
bool promote_co2_warmup();
uint32_t co2_activity_generation();
bool co2_warmup_active();
bool co2_bus_powered_down();

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
