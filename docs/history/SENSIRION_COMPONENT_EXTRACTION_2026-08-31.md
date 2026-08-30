# Sensirion bridge component extraction (2026-08-31)

The Sensirion/MyAmbience implementation is now an independent ESPHome external
component under `sensirion_gadget_bridge/`. It owns the authoritative
profile-aware sample, sequential-source coalescing, live advertising, GATT
services, settings, persistent history and OTA flushing. The mobile AHT21
variants compose this component directly and no longer instantiate the Unni
orchestrator merely to obtain BLE/history.

The Unni firmware requires and composes the same bridge and supplies decoded physical-air
T/RH and direct CO2 samples through its C++ interface. Existing flash metadata,
sample width, history wire protocol, BLE identity and bonds are not migrated.
The old `co2_monitor_0601` Sensirion setters remain as a compatibility path,
but shipped BLE-enabled Unni YAMLs use `sensirion_bridge_id`.

History transport owns generic connection state, cursor retention, CCCD state,
packet cadence and total/no-progress watchdogs. It accepts an optional
`HistoryTransferGuard`; standalone/mobile use no guard. Unni injects
`UnniHistoryTransferGuard`, which alone knows native CO2/RT-RH prediction,
active capture probes and adaptive drain lead time. This keeps the transport
reusable without weakening the timing protection required by the passive
sniffer.

The local `sensirion/` tree was used only as a protocol reference. It is ignored
at the repository root and is neither copied, compiled nor linked.

ENS160 remains outside the bridge. TVOC and AQI are Home Assistant-only, and
estimated eCO2 is never treated as direct CO2 or routed into BLE, GATT or
history. Both mobile variants therefore drive history exclusively from the
AHT21 temperature/humidity source IDs, regardless of ENS160 presence or state.
