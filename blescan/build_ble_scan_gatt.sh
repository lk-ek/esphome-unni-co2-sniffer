#!/bin/bash
set -euo pipefail

APP="BLEScanGatt.app"
BIN="$APP/Contents/MacOS/BLEScanGatt"

rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS"

cat > "$APP/Contents/Info.plist" <<'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"
  "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key>
  <string>BLEScanGatt</string>
  <key>CFBundleIdentifier</key>
  <string>local.unni.blescangatt</string>
  <key>CFBundleName</key>
  <string>BLEScanGatt</string>
  <key>CFBundlePackageType</key>
  <string>APPL</string>
  <key>CFBundleVersion</key>
  <string>1</string>
  <key>CFBundleShortVersionString</key>
  <string>1.0</string>
  <key>NSBluetoothAlwaysUsageDescription</key>
  <string>Inspect BLE GATT services of the Unni-CO2 sensor.</string>
</dict>
</plist>
PLIST

clang++ \
  -std=c++17 \
  -fobjc-arc \
  -framework Foundation \
  -framework CoreBluetooth \
  ble_scan_gatt.mm \
  -o "$BIN"

echo "Built $APP"
echo "Run:"
echo "  ./$BIN"
