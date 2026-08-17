<!-- SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Third-party notices and provenance

This repository is licensed as described in [LICENSE](LICENSE). Some protocol-compatibility code was developed with reference to Sensirion's open-source Gadget BLE and UPT Core implementations. The applicable upstream BSD-3-Clause notices are preserved in `LICENSES/`.

## Sensirion UPT Core 0.5.1

Upstream project: Sensirion UPT Core, version 0.5.1.

License: BSD-3-Clause. Copyright 2024 Sensirion AG. See [LICENSES/Sensirion-UPT-Core-0.5.1-BSD-3-Clause.txt](LICENSES/Sensirion-UPT-Core-0.5.1-BSD-3-Clause.txt).

Relevant local code:

- `co2_monitor_0601/sensirion_sample.h`: the temperature/humidity sample encoding formulas and `T_RH_CO2_ALT` byte layout are adapted from / compatible with Sensirion UPT Core's BLE protocol implementation.

## Sensirion Gadget BLE Arduino Library 1.5.0

Upstream project: Sensirion Gadget BLE Arduino Library, version 1.5.0.

License: BSD-3-Clause. Copyright (c) 2020, Sensirion AG. See [LICENSES/Sensirion-Gadget-BLE-1.5.0-BSD-3-Clause.txt](LICENSES/Sensirion-Gadget-BLE-1.5.0-BSD-3-Clause.txt).

Relevant local code:

- `co2_monitor_0601/sensirion_ble.cpp`: Gadget/MyAmbience-compatible advertising layout and device-identity behavior are implemented with reference to the upstream Gadget BLE behavior.
- `co2_monitor_0601/sensirion_history.cpp`: Gadget/MyAmbience-compatible GATT UUID topology and history-download wire format are implemented with reference to the upstream Gadget BLE implementation.

The local RAM/flash history ring, persistence/journaling, power policy, passive sensor decoders, ESPHome integration, and orchestration are project-specific implementations and are not claimed to originate from Sensirion Gadget BLE.

## Sensirion SHT43 DemoBoard BLE Firmware

Upstream project: Sensirion SHT43 DemoBoard BLE Firmware.

License: BSD-3-Clause. Copyright (c) 2023, Sensirion AG. See [LICENSES/Sensirion-SHT43-DemoBoard-BLE-Firmware-BSD-3-Clause.txt](LICENSES/Sensirion-SHT43-DemoBoard-BLE-Firmware-BSD-3-Clause.txt).

Relevant local code:

- `co2_monitor_0601/sensirion_settings.cpp`: the Sensirion-compatible Device Settings service UUID topology, selected SHT43 DemoBoard settings, and authenticated/encrypted GATT security model are implemented with reference to the upstream SHT43 DemoBoard firmware.
- `co2_monitor_0601/sensirion_sht43_probe.cpp`: the experimental SHT43 identity-probe GATT topology is implemented with reference to the upstream SHT43 DemoBoard firmware.
- `co2_monitor_0601/sensirion_ble.cpp`: the experimental SHT43 identity/advertising mode and SHT43-compatible sample encoding are implemented with reference to the upstream SHT43 DemoBoard firmware.

The local ESP-IDF/ESPHome implementation is project-specific. No STM32CubeWB source from the upstream SHT43 DemoBoard firmware is vendored or linked by this project.

## Sensirion CO2 wire protocol

`co2_monitor_0601/co2_decoder.cpp` implements the CRC-8 parameters and frame semantics used by the observed SCD4x-compatible CO2 transaction. This is recorded as protocol provenance; it is not treated as a vendored Sensirion driver or library dependency.

## Runtime dependencies

The production component does not vendor or directly link the Sensirion Gadget BLE Arduino Library or Sensirion UPT Core. It implements the compatible wire formats using ESPHome/ESP-IDF BLE APIs. Runtime framework dependencies retain their own upstream licenses.

The standalone Sensirion reference test used during development is separate from the production component and directly depends on Sensirion Gadget BLE, Sensirion UPT Core, and NimBLE-Arduino; those dependencies are not vendored in this repository.

## No affiliation

Compatibility with Sensirion Gadget/MyAmbience protocols does not imply affiliation with, endorsement by, or manufacture by Sensirion AG.
