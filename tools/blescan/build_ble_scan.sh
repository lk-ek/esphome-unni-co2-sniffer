#!/bin/bash
# SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

APP="BLEScan.app"
BIN="$APP/Contents/MacOS/BLEScan"

rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS"

cat > "$APP/Contents/Info.plist" <<'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"
  "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key>
  <string>BLEScan</string>
  <key>CFBundleIdentifier</key>
  <string>local.unni.blescan</string>
  <key>CFBundleName</key>
  <string>BLEScan</string>
  <key>CFBundlePackageType</key>
  <string>APPL</string>
  <key>CFBundleVersion</key>
  <string>1</string>
  <key>CFBundleShortVersionString</key>
  <string>1.0</string>
  <key>NSBluetoothAlwaysUsageDescription</key>
  <string>Scan nearby BLE advertisements for debugging the Unni-CO2 sensor.</string>
</dict>
</plist>
PLIST

clang++ \
  -std=c++17 \
  -fobjc-arc \
  -framework Foundation \
  -framework CoreBluetooth \
  ble_scan.mm \
  -o "$BIN"

echo "Built $APP"
echo
echo "Run filtered scan:"
echo "  ./$BIN"
echo
echo "Run unfiltered scan:"
echo "  ./$BIN --all"
