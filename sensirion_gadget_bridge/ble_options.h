// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Compile-time feature switches retained under their established names for
// wire/build compatibility with the Unni component.
//
// ESPHome's cg.add_define() writes these feature flags to
// esphome/core/defines.h.  Include that file BEFORE applying standalone
// defaults; otherwise a translation unit such as sensirion_ble.cpp sees the
// defaults first and incorrectly compiles BLE code even for ble:false.
#include "esphome/core/defines.h"

// Fallbacks are only for building the component sources outside ESPHome
// codegen.  ESPHome-generated builds already have the three defines above.
#ifndef UNNI_BLE_ENABLED
#ifdef USE_HOST
#define UNNI_BLE_ENABLED 0
#else
#define UNNI_BLE_ENABLED 1
#endif
#endif

#ifndef UNNI_BLE_LIVE_ENABLED
#ifdef USE_HOST
#define UNNI_BLE_LIVE_ENABLED 0
#else
#define UNNI_BLE_LIVE_ENABLED UNNI_BLE_ENABLED
#endif
#endif

#ifndef UNNI_BLE_HISTORY_ENABLED
#ifdef USE_HOST
#define UNNI_BLE_HISTORY_ENABLED 0
#else
#define UNNI_BLE_HISTORY_ENABLED UNNI_BLE_ENABLED
#endif
#endif

#ifndef UNNI_RUNTIME_DIAGNOSTICS
#define UNNI_RUNTIME_DIAGNOSTICS 0
#endif

#ifndef UNNI_BLE_DEVICE_DERIVED_IDENTITY
#define UNNI_BLE_DEVICE_DERIVED_IDENTITY 1
#endif

#if UNNI_BLE_LIVE_ENABLED && !UNNI_BLE_ENABLED
#error "UNNI_BLE_LIVE_ENABLED requires UNNI_BLE_ENABLED"
#endif

#if UNNI_BLE_HISTORY_ENABLED && !UNNI_BLE_ENABLED
#error "UNNI_BLE_HISTORY_ENABLED requires UNNI_BLE_ENABLED"
#endif
