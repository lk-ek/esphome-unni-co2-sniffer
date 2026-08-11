#pragma once

// Compile-time feature switches for the Unni CO2 sniffer.
//
// They are normally emitted by the ESPHome component from the YAML options
// `ble`, `ble_live`, and `ble_history`.  Defaults keep the historical behaviour
// when the sources are built outside ESPHome codegen.

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
