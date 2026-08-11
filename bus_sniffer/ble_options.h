// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Compile-time feature switches for the Unni CO2 sniffer.
//
// ESPHome's cg.add_define() writes these feature flags to
// esphome/core/defines.h.  Include that file BEFORE applying standalone
// defaults; otherwise a translation unit such as sensirion_ble.cpp sees the
// defaults first and incorrectly compiles BLE code even for ble:false.
#include "esphome/core/defines.h"

// Fallbacks are only for building the component sources outside ESPHome
// codegen.  ESPHome-generated builds already have the three defines above.
#ifndef UNNI_BLE_ENABLED
#define UNNI_BLE_ENABLED 1
#endif

#ifndef UNNI_BLE_LIVE_ENABLED
#define UNNI_BLE_LIVE_ENABLED UNNI_BLE_ENABLED
#endif

#ifndef UNNI_BLE_HISTORY_ENABLED
#define UNNI_BLE_HISTORY_ENABLED UNNI_BLE_ENABLED
#endif

#if UNNI_BLE_LIVE_ENABLED && !UNNI_BLE_ENABLED
#error "UNNI_BLE_LIVE_ENABLED requires UNNI_BLE_ENABLED"
#endif

#if UNNI_BLE_HISTORY_ENABLED && !UNNI_BLE_ENABLED
#error "UNNI_BLE_HISTORY_ENABLED requires UNNI_BLE_ENABLED"
#endif
