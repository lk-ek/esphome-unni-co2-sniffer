# SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
# SPDX-License-Identifier: GPL-3.0-or-later
import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import esp32_ble, esp32_ble_server, sensor, switch, time as time_component
from esphome.components.esp32 import add_idf_sdkconfig_option, add_partition
from esphome.config_helpers import filter_source_files_from_platform
from esphome.const import CONF_ID, PlatformFramework
from esphome.core import CORE, TimePeriod

DEPENDENCIES = []


def AUTO_LOAD(config):
    loads = ["sensor", "switch"]
    if config.get(CONF_BLE, True) and not CORE.is_host:
        loads.append("esp32_ble_server")
    return loads


FILTER_SOURCE_FILES = filter_source_files_from_platform(
    {
        "sensirion_gadget_bridge.cpp": {PlatformFramework.ESP32_IDF},
        "sensirion_gadget_bridge_host.cpp": {PlatformFramework.HOST_NATIVE},
        "sensirion_ble.cpp": {PlatformFramework.ESP32_IDF},
        "sensirion_history.cpp": {PlatformFramework.ESP32_IDF},
        "sensirion_settings.cpp": {PlatformFramework.ESP32_IDF},
        "sensirion_sht43_probe.cpp": {PlatformFramework.ESP32_IDF},
    }
)

CONF_BLE = "ble"
CONF_BLE_LIVE = "ble_live"
CONF_BLE_HISTORY = "ble_history"
CONF_BLE_ID = "ble_id"
CONF_BLE_SERVER_ID = "ble_server_id"
CONF_DEVICE_NAME = "device_name"
CONF_IDENTITY_MODE = "identity_mode"
CONF_ADVERTISING_INTERVAL = "advertising_interval"
CONF_HISTORY_TIME_ID = "history_time_id"
CONF_PROFILE = "profile"
CONF_TEMPERATURE_ID = "temperature_id"
CONF_HUMIDITY_ID = "humidity_id"
CONF_SHT43_IDENTITY_PROBE = "sht43_identity_probe"
CONF_PAIRING_MODE = "pairing_mode"
CONF_PAIRING_WINDOW = "pairing_window"

bridge_ns = cg.esphome_ns.namespace("co2_monitor_0601")
SensirionGadgetBridge = bridge_ns.class_("SensirionGadgetBridge", cg.Component)
SensirionPairingModeSwitch = bridge_ns.class_("SensirionPairingModeSwitch", switch.Switch)
SensirionProfile = bridge_ns.enum("SensirionProfile", is_class=True)


def _device_name(value):
    value = cv.string_strict(value)
    if len(value.encode("utf-8")) > 31:
        raise cv.Invalid("device_name must be at most 31 UTF-8 bytes")
    return value


def _validate(config):
    have_temperature = CONF_TEMPERATURE_ID in config
    have_humidity = CONF_HUMIDITY_ID in config
    if have_temperature != have_humidity:
        raise cv.Invalid("temperature_id and humidity_id must be configured together")
    explicit_profile = config.get(CONF_PROFILE)
    if config.get(CONF_SHT43_IDENTITY_PROBE, False) and explicit_profile not in (None, "sht43_trh"):
        raise cv.Invalid("sht43_identity_probe: true conflicts with profile: trh_co2")
    config[CONF_PROFILE] = explicit_profile or (
        "sht43_trh" if config.get(CONF_SHT43_IDENTITY_PROBE, False) else "trh_co2"
    )
    if config[CONF_BLE_LIVE] and not config[CONF_BLE]:
        raise cv.Invalid("ble_live: true requires ble: true")
    if config[CONF_BLE_HISTORY] and not config[CONF_BLE]:
        raise cv.Invalid("ble_history: true requires ble: true")
    if CORE.is_host or not config[CONF_BLE]:
        config.pop(CONF_BLE_ID, None)
        config.pop(CONF_BLE_SERVER_ID, None)
    if CORE.is_host:
        config.pop(CONF_HISTORY_TIME_ID, None)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SensirionGadgetBridge),
            cv.Optional(CONF_BLE, default=True): cv.boolean,
            cv.Optional(CONF_BLE_LIVE, default=True): cv.boolean,
            cv.Optional(CONF_BLE_HISTORY, default=True): cv.boolean,
            cv.Optional(CONF_DEVICE_NAME, default="Unni CO2 Monitor"): _device_name,
            cv.Optional(CONF_IDENTITY_MODE, default="device_derived"): cv.one_of(
                "legacy_fixed", "device_derived", lower=True
            ),
            cv.Optional(CONF_ADVERTISING_INTERVAL, default="2s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=TimePeriod(milliseconds=20), max=TimePeriod(milliseconds=10240)),
            ),
            cv.Optional(CONF_HISTORY_TIME_ID): cv.use_id(time_component.RealTimeClock),
            cv.Optional(CONF_PROFILE): cv.one_of("trh_co2", "sht43_trh", lower=True),
            cv.Optional(CONF_SHT43_IDENTITY_PROBE, default=False): cv.boolean,
            cv.Optional(CONF_TEMPERATURE_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_HUMIDITY_ID): cv.use_id(sensor.Sensor),
            cv.GenerateID(CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
            cv.GenerateID(CONF_BLE_SERVER_ID): cv.use_id(esp32_ble_server.BLEServer),
            cv.Optional(CONF_PAIRING_MODE): switch.switch_schema(
                SensirionPairingModeSwitch, icon="mdi:bluetooth-connect"
            ),
            cv.Optional(CONF_PAIRING_WINDOW, default="60s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=TimePeriod(milliseconds=10000), max=TimePeriod(milliseconds=3600000)),
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    ble_enabled = config[CONF_BLE]
    # The profile is authoritative. sht43_identity_probe is only a legacy
    # spelling that selects the same profile during validation.
    probe = config[CONF_PROFILE] == "sht43_trh"
    cg.add_define("UNNI_BLE_ENABLED", int(ble_enabled))
    cg.add_define("UNNI_BLE_LIVE_ENABLED", int(config[CONF_BLE_LIVE]))
    cg.add_define("UNNI_BLE_HISTORY_ENABLED", int(config[CONF_BLE_HISTORY]))
    cg.add_define("UNNI_SHT43_IDENTITY_PROBE", int(probe))
    cg.add_define("UNNI_BLE_DEVICE_DERIVED_IDENTITY", int(config[CONF_IDENTITY_MODE] == "device_derived"))

    profile = SensirionProfile.SHT43_TRH if config[CONF_PROFILE] == "sht43_trh" else SensirionProfile.TRH_CO2
    cg.add(var.set_profile(profile))
    cg.add(var.set_ble_device_name(config[CONF_DEVICE_NAME]))
    cg.add(var.set_advertising_interval(config[CONF_ADVERTISING_INTERVAL]))
    cg.add(var.set_pairing_window(config[CONF_PAIRING_WINDOW]))

    if CONF_TEMPERATURE_ID in config:
        cg.add(var.set_temperature_source(await cg.get_variable(config[CONF_TEMPERATURE_ID])))
        cg.add(var.set_humidity_source(await cg.get_variable(config[CONF_HUMIDITY_ID])))
    if CONF_HISTORY_TIME_ID in config and not CORE.is_host:
        cg.add(var.set_history_time(await cg.get_variable(config[CONF_HISTORY_TIME_ID])))
    if CONF_PAIRING_MODE in config:
        pairing = await switch.new_switch(config[CONF_PAIRING_MODE])
        cg.add(pairing.set_parent(var))
        cg.add(var.set_pairing_switch(pairing))

    if CORE.is_host:
        return

    if config[CONF_BLE_HISTORY]:
        add_partition("senshist", "data", "spiffs", 0x10000)
    if ble_enabled:
        add_idf_sdkconfig_option("CONFIG_BT_CTRL_MODEM_SLEEP", True)
        add_idf_sdkconfig_option("CONFIG_BT_CTRL_MODEM_SLEEP_MODE_1", True)
        add_idf_sdkconfig_option("CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL", True)
        add_idf_sdkconfig_option("CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP", True)
        ble = await cg.get_variable(config[CONF_BLE_ID])
        cg.add(ble.set_name("SHT43 DB" if probe else "S"))
        esp32_ble.register_gatts_event_handler(ble, var)
        esp32_ble.register_gap_event_handler(ble, var)
        cg.add(var.set_gatt_server(await cg.get_variable(config[CONF_BLE_SERVER_ID])))
