# SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
# SPDX-License-Identifier: GPL-3.0-or-later
import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import binary_sensor, esp32_ble, esp32_ble_server, sensor, switch
from esphome.components.esp32 import (
    add_idf_sdkconfig_option,
    add_partition,
)
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.core import TimePeriod

# BLE is deliberately NOT a hard dependency. A real `ble: false` build therefore
# does not need esp32_ble / esp32_ble_server in the YAML at all.
DEPENDENCIES = []


def AUTO_LOAD(config):
    loads = ["sensor", "binary_sensor", "switch"]
    if config.get(CONF_BLE, True):
        loads.append("esp32_ble_server")
    return loads

CONF_CO2 = "co2"
CONF_CRC_ERRORS = "crc_errors"
CONF_FRAME_ERRORS = "frame_errors"
CONF_RT_TEMPERATURE = "rt_temperature"
CONF_RH_HUMIDITY = "rh_humidity"
CONF_BLE = "ble"
CONF_BLE_LIVE = "ble_live"
CONF_BLE_HISTORY = "ble_history"
CONF_BLE_ID = "ble_id"
CONF_BLE_SERVER_ID = "ble_server_id"
CONF_BLE_ADVERTISING_INTERVAL = "ble_advertising_interval"
CONF_BLE_BATTERY_ADVERTISING_INTERVAL = "ble_battery_advertising_interval"
CONF_HA_PUBLISH_INTERVAL = "ha_publish_interval"
CONF_HOME_ASSISTANT = "home_assistant"
CONF_SNIFFER_ENABLED = "sniffer_enabled"
CONF_RTRH_ENABLED = "rtrh_enabled"
CONF_RTRH_GPIO_SETUP = "rtrh_gpio_setup"
CONF_RTRH_EDGE_CAPTURE = "rtrh_edge_capture"
CONF_RTRH_DECODE_ONLY = "rtrh_decode_only"
CONF_SNIFFER_START_DELAY = "sniffer_start_delay"
CONF_DEBUG_METRICS = "debug_metrics"
CONF_DEBUG_CAPTURE = "debug_capture"
CONF_LIGHT_SLEEP = "light_sleep"
CONF_LIGHT_SLEEP_MAX_AWAKE = "light_sleep_max_awake"
CONF_RT_PIN = "rt_pin"
CONF_RH_PIN = "rh_pin"
CONF_CO2_SDA_PIN = "co2_sda_pin"
CONF_CO2_SCL_PIN = "co2_scl_pin"
CONF_BATTERY_PIN = "battery_pin"
CONF_BATTERY_UPDATE_INTERVAL = "battery_update_interval"
CONF_BATTERY_DIVIDER_RATIO = "battery_divider_ratio"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_BATTERY_LEVEL = "battery_level"
CONF_USB_POWER_PIN = "usb_power_pin"
CONF_USB_POWER = "usb_power"
CONF_ENERGY_SAVE_MODE = "energy_save_mode"
CONF_BLE_PAIRING_MODE = "ble_pairing_mode"
CONF_BLE_PAIRING_WINDOW = "ble_pairing_window"
CONF_SHT43_IDENTITY_PROBE = "sht43_identity_probe"
CONF_ENERGY_SAVE_MODE_DEFAULT = "energy_save_mode_default"
CONF_ENERGY_SAVE_GRACE = "energy_save_grace"
CONF_THERMAL_TRANSIENT_ON_RATE = "thermal_transient_on_rate"
CONF_THERMAL_TRANSIENT_OFF_RATE = "thermal_transient_off_rate"

CONF_REF_PERIOD = "ref_period"
CONF_RT_PERIOD = "rt_period"
CONF_RH_STATE_PERIOD = "rh_state_period"
CONF_RT_RATIO = "rt_ratio"
CONF_RH_RATIO = "rh_ratio"
CONF_RH_LOG = "rh_log"
CONF_MEASUREMENT_QUALITY = "measurement_quality"
CONF_THERMAL_TRANSIENT = "thermal_transient"
CONF_TEMPERATURE_EXTRAPOLATION = "temperature_extrapolation"
CONF_HUMIDITY_EXTRAPOLATION = "humidity_extrapolation"
CONF_CALIBRATION_EXTRAPOLATION = "calibration_extrapolation"

co2_monitor_0601_ns = cg.esphome_ns.namespace("co2_monitor_0601")
CO2Monitor0601 = co2_monitor_0601_ns.class_("CO2Monitor0601", cg.Component)
EnergySaveModeSwitch = co2_monitor_0601_ns.class_("EnergySaveModeSwitch", switch.Switch)
BlePairingModeSwitch = co2_monitor_0601_ns.class_("BlePairingModeSwitch", switch.Switch)


def _sensor_schema(**kwargs):
    return sensor.sensor_schema(**kwargs)


def _diagnostic_sensor(**kwargs):
    return _sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC, **kwargs)


def _diagnostic_binary_sensor():
    return binary_sensor.binary_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC)


# YAML key -> (schema, CO2Monitor0601 setter)
SENSOR_OUTPUTS = {
    CONF_CO2: (
        _sensor_schema(
            unit_of_measurement="ppm",
            accuracy_decimals=0,
            device_class="carbon_dioxide",
            state_class="measurement",
        ),
        "set_co2_sensor",
    ),
    CONF_CRC_ERRORS: (
        _diagnostic_sensor(accuracy_decimals=0, state_class="total_increasing"),
        "set_crc_errors_sensor",
    ),
    CONF_FRAME_ERRORS: (
        _diagnostic_sensor(accuracy_decimals=0, state_class="total_increasing"),
        "set_frame_errors_sensor",
    ),
    CONF_RT_TEMPERATURE: (
        _sensor_schema(
            unit_of_measurement="°C",
            accuracy_decimals=1,
            device_class="temperature",
            state_class="measurement",
        ),
        "set_rt_temperature_sensor",
    ),
    CONF_RH_HUMIDITY: (
        _sensor_schema(
            unit_of_measurement="%",
            accuracy_decimals=1,
            device_class="humidity",
            state_class="measurement",
        ),
        "set_rh_humidity_sensor",
    ),
    CONF_BATTERY_VOLTAGE: (
        _sensor_schema(
            unit_of_measurement="V",
            accuracy_decimals=3,
            device_class="voltage",
            state_class="measurement",
        ),
        "set_battery_voltage_sensor",
    ),
    CONF_BATTERY_LEVEL: (
        _sensor_schema(
            unit_of_measurement="%",
            accuracy_decimals=0,
            device_class="battery",
            state_class="measurement",
        ),
        "set_battery_level_sensor",
    ),
    CONF_REF_PERIOD: (
        _diagnostic_sensor(unit_of_measurement="µs", accuracy_decimals=3, state_class="measurement"),
        "set_ref_period_sensor",
    ),
    CONF_RT_PERIOD: (
        _diagnostic_sensor(unit_of_measurement="µs", accuracy_decimals=3, state_class="measurement"),
        "set_rt_period_sensor",
    ),
    CONF_RH_STATE_PERIOD: (
        _diagnostic_sensor(unit_of_measurement="µs", accuracy_decimals=1, state_class="measurement"),
        "set_rh_state_period_sensor",
    ),
    CONF_RT_RATIO: (
        _diagnostic_sensor(accuracy_decimals=6, state_class="measurement"),
        "set_rt_ratio_sensor",
    ),
    CONF_RH_RATIO: (
        _diagnostic_sensor(accuracy_decimals=6, state_class="measurement"),
        "set_rh_ratio_sensor",
    ),
    CONF_RH_LOG: (
        _diagnostic_sensor(accuracy_decimals=6, state_class="measurement"),
        "set_rh_log_sensor",
    ),
    CONF_MEASUREMENT_QUALITY: (
        _diagnostic_sensor(unit_of_measurement="%", accuracy_decimals=0, state_class="measurement"),
        "set_measurement_quality_sensor",
    ),
}

BINARY_OUTPUTS = {
    CONF_USB_POWER: "set_usb_power_sensor",
    CONF_THERMAL_TRANSIENT: "set_thermal_transient_sensor",
    CONF_TEMPERATURE_EXTRAPOLATION: "set_temperature_extrapolation_sensor",
    CONF_HUMIDITY_EXTRAPOLATION: "set_humidity_extrapolation_sensor",
    CONF_CALIBRATION_EXTRAPOLATION: "set_calibration_extrapolation_sensor",
}


def _validate_features(config):
    # `home_assistant` is deliberately opt-out. Materialize the default here as
    # well as in cv.Optional so every later validation/codegen path sees True
    # when the key is omitted.
    config.setdefault(CONF_HOME_ASSISTANT, True)

    pins = [config[CONF_RT_PIN], config[CONF_RH_PIN], config[CONF_CO2_SDA_PIN], config[CONF_CO2_SCL_PIN], config[CONF_BATTERY_PIN], config[CONF_USB_POWER_PIN]]
    if len(set(pins)) != len(pins):
        raise cv.Invalid("RT/RH, CO2, battery and USB-power GPIOs must be unique")
    if config[CONF_BATTERY_PIN] not in range(0, 5):
        raise cv.Invalid("battery_pin must be an ESP32-C3 ADC1 GPIO (0..4)")

    if config[CONF_BLE_LIVE] and not config[CONF_BLE]:
        raise cv.Invalid("ble_live: true requires ble: true")
    if config[CONF_BLE_HISTORY] and not config[CONF_BLE]:
        raise cv.Invalid("ble_history: true requires ble: true")
    if config.get(CONF_SHT43_IDENTITY_PROBE, False) and not config[CONF_BLE]:
        raise cv.Invalid("sht43_identity_probe: true requires ble: true")

    if not config[CONF_BLE]:
        config.pop(CONF_BLE_ID, None)
        config.pop(CONF_BLE_SERVER_ID, None)
        config.pop(CONF_BLE_PAIRING_MODE, None)

    if not config[CONF_HOME_ASSISTANT]:
        for key in SENSOR_OUTPUTS:
            config.pop(key, None)
        for key in BINARY_OUTPUTS:
            config.pop(key, None)
        config.pop(CONF_ENERGY_SAVE_MODE, None)
        config.pop(CONF_BLE_PAIRING_MODE, None)

    if not config[CONF_DEBUG_METRICS]:
        for key in DEBUG_SENSOR_DEFAULTS:
            config.pop(key, None)
        for key in DEBUG_BINARY_DEFAULTS:
            config.pop(key, None)

    return config


_SCHEMA = {
    cv.GenerateID(): cv.declare_id(CO2Monitor0601),
    cv.Optional(CONF_BLE, default=True): cv.boolean,
    cv.Optional(CONF_BLE_LIVE, default=True): cv.boolean,
    cv.Optional(CONF_BLE_HISTORY, default=True): cv.boolean,
    cv.GenerateID(CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
    cv.GenerateID(CONF_BLE_SERVER_ID): cv.use_id(esp32_ble_server.BLEServer),
    cv.Optional(CONF_BLE_ADVERTISING_INTERVAL, default="2s"): cv.All(
        cv.positive_time_period_milliseconds,
        cv.Range(min=TimePeriod(milliseconds=20), max=TimePeriod(milliseconds=10240)),
    ),
    cv.Optional(CONF_BLE_BATTERY_ADVERTISING_INTERVAL, default="5s"): cv.All(
        cv.positive_time_period_milliseconds,
        cv.Range(min=TimePeriod(milliseconds=20), max=TimePeriod(milliseconds=10240)),
    ),
    cv.Optional(CONF_HA_PUBLISH_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_HOME_ASSISTANT, default=True): cv.boolean,
    cv.Optional(CONF_SHT43_IDENTITY_PROBE, default=False): cv.boolean,
    cv.Optional(CONF_SNIFFER_ENABLED, default=True): cv.boolean,
    cv.Optional(CONF_RTRH_ENABLED, default=True): cv.boolean,
    cv.Optional(CONF_RTRH_GPIO_SETUP, default=False): cv.boolean,
    cv.Optional(CONF_RTRH_EDGE_CAPTURE, default=False): cv.boolean,
    cv.Optional(CONF_RTRH_DECODE_ONLY, default=False): cv.boolean,
    cv.Optional(CONF_SNIFFER_START_DELAY, default="0s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_DEBUG_METRICS, default=False): cv.boolean,
    cv.Optional(CONF_DEBUG_CAPTURE, default=False): cv.boolean,
    cv.Optional(CONF_LIGHT_SLEEP, default=True): cv.boolean,
    cv.Optional(CONF_LIGHT_SLEEP_MAX_AWAKE, default="10s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_RT_PIN, default=3): cv.int_range(min=0, max=21),
    cv.Optional(CONF_RH_PIN, default=4): cv.int_range(min=0, max=21),
    cv.Optional(CONF_CO2_SDA_PIN, default=6): cv.int_range(min=0, max=21),
    cv.Optional(CONF_CO2_SCL_PIN, default=7): cv.int_range(min=0, max=21),
    cv.Optional(CONF_BATTERY_PIN, default=2): cv.int_range(min=0, max=4),
    cv.Optional(CONF_BATTERY_UPDATE_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_BATTERY_DIVIDER_RATIO, default=2.0): cv.float_range(min=1.0, max=20.0),
    cv.Optional(CONF_USB_POWER_PIN, default=5): cv.int_range(min=0, max=21),
    cv.Optional(CONF_ENERGY_SAVE_MODE_DEFAULT, default=False): cv.boolean,
    cv.Optional(CONF_ENERGY_SAVE_GRACE, default="3s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_ENERGY_SAVE_MODE, default={"name": "Energy Save Mode", "icon": "mdi:leaf"}): switch.switch_schema(EnergySaveModeSwitch),
    cv.Optional(CONF_BLE_PAIRING_MODE, default={"name": "BLE Pairing Mode", "icon": "mdi:bluetooth-connect"}): switch.switch_schema(BlePairingModeSwitch),
    cv.Optional(CONF_BLE_PAIRING_WINDOW, default="60s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_THERMAL_TRANSIENT_ON_RATE, default=0.8): cv.float_range(min=0.05, max=20.0),
    cv.Optional(CONF_THERMAL_TRANSIENT_OFF_RATE, default=0.3): cv.float_range(min=0.01, max=20.0),
}

PRIMARY_SENSOR_DEFAULTS = {
    CONF_CO2: {"name": "CO2", "icon": "mdi:molecule-co2"},
    CONF_RT_TEMPERATURE: {"name": "RT Temperature", "icon": "mdi:thermometer"},
    CONF_RH_HUMIDITY: {"name": "RH Humidity", "icon": "mdi:water-percent"},
    CONF_BATTERY_VOLTAGE: {"name": "Battery Voltage", "icon": "mdi:battery"},
    CONF_BATTERY_LEVEL: {"name": "Battery Level", "icon": "mdi:battery"},
}

DEBUG_SENSOR_DEFAULTS = {
    CONF_CRC_ERRORS: {"name": "CO2 Sniffer CRC Errors"},
    CONF_FRAME_ERRORS: {"name": "CO2 Sniffer Frame Errors"},
    CONF_REF_PERIOD: {"name": "RT RH REF Period"},
    CONF_RT_PERIOD: {"name": "RT RH RT Period"},
    CONF_RH_STATE_PERIOD: {"name": "RT RH RH State Period"},
    CONF_RT_RATIO: {"name": "RT RH RT Ratio"},
    CONF_RH_RATIO: {"name": "RT RH RH Ratio"},
    CONF_RH_LOG: {"name": "RT RH RH Log Ratio"},
    CONF_MEASUREMENT_QUALITY: {"name": "RT RH Measurement Quality"},
}

DEBUG_BINARY_DEFAULTS = {
    CONF_THERMAL_TRANSIENT: {"name": "RT RH Thermal Transient"},
    CONF_TEMPERATURE_EXTRAPOLATION: {"name": "RT RH Temperature Extrapolation"},
    CONF_HUMIDITY_EXTRAPOLATION: {"name": "RT RH Humidity Extrapolation"},
    CONF_CALIBRATION_EXTRAPOLATION: {"name": "RT RH Calibration Extrapolation"},
}

for key, (schema, _) in SENSOR_OUTPUTS.items():
    if key in PRIMARY_SENSOR_DEFAULTS:
        _SCHEMA[cv.Optional(key, default=PRIMARY_SENSOR_DEFAULTS[key])] = schema
    elif key in DEBUG_SENSOR_DEFAULTS:
        _SCHEMA[cv.Optional(key, default=DEBUG_SENSOR_DEFAULTS[key])] = schema
    else:
        _SCHEMA[cv.Optional(key)] = schema
for key in BINARY_OUTPUTS:
    if key == CONF_USB_POWER:
        _SCHEMA[cv.Optional(key, default={"name": "USB Power", "device_class": "power"})] = binary_sensor.binary_sensor_schema()
    else:
        _SCHEMA[cv.Optional(key, default=DEBUG_BINARY_DEFAULTS[key])] = _diagnostic_binary_sensor()

CONFIG_SCHEMA = cv.All(cv.Schema(_SCHEMA).extend(cv.COMPONENT_SCHEMA), _validate_features)


async def _configure_outputs(var, config):
    for key, (_, setter_name) in SENSOR_OUTPUTS.items():
        if key not in config:
            continue
        output = await sensor.new_sensor(config[key])
        cg.add(getattr(var, setter_name)(output))

    for key, setter_name in BINARY_OUTPUTS.items():
        if key not in config:
            continue
        output = await binary_sensor.new_binary_sensor(config[key])
        cg.add(getattr(var, setter_name)(output))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    ble_enabled = config[CONF_BLE]
    home_assistant_enabled = config[CONF_HOME_ASSISTANT]
    sht43_identity_probe = config.get(CONF_SHT43_IDENTITY_PROBE, False)

    # This component is timing-sensitive and validated at 80 MHz on ESP32-C3.
    # Keep that platform detail out of user YAML.
    add_idf_sdkconfig_option("CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_80", True)
    add_idf_sdkconfig_option("CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_160", False)

    if config[CONF_BLE_HISTORY]:
        add_partition("senshist", "data", "spiffs", 0x10000)

    if config[CONF_LIGHT_SLEEP]:
        add_idf_sdkconfig_option("CONFIG_PM_ENABLE", True)
        add_idf_sdkconfig_option("CONFIG_FREERTOS_USE_TICKLESS_IDLE", True)
        add_idf_sdkconfig_option("CONFIG_ESP_PHY_MAC_BB_PD", True)

    if ble_enabled:
        add_idf_sdkconfig_option("CONFIG_BT_CTRL_MODEM_SLEEP", True)
        add_idf_sdkconfig_option("CONFIG_BT_CTRL_MODEM_SLEEP_MODE_1", True)
        add_idf_sdkconfig_option("CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL", True)
        add_idf_sdkconfig_option("CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP", True)

    # Child sensor entities are created from this component schema rather than
    # from a top-level `sensor:` platform entry. Ensure the core/API/web-server
    # sensor domain is compiled in so App-registered child sensors are exposed.
    # The sensor/switch framework remains linked because the runtime source is
    # shared between normal and BLE-only builds. In BLE-only mode no entities
    # are instantiated and no API/Wi-Fi components are present in the shipped
    # build YAML.
    cg.add_define("USE_SENSOR")
    cg.add_define("USE_BINARY_SENSOR")
    cg.add_define("USE_SWITCH")

    # ESPHome 2026.8 sizes the per-domain App entity vectors from generated
    # ESPHOME_ENTITY_*_COUNT defines. In a BLE-only build these framework
    # domains are still referenced by the shared C++ type declarations, but
    # no entities are registered, so core codegen has no count to emit.
    # Define the intentional zero-entity case explicitly; this does not create
    # entities and does not pull in API/Wi-Fi.
    if not home_assistant_enabled:
        cg.add_define("ESPHOME_ENTITY_SENSOR_COUNT", 0)
        cg.add_define("ESPHOME_ENTITY_BINARY_SENSOR_COUNT", 0)
        cg.add_define("ESPHOME_ENTITY_SWITCH_COUNT", 0)

    cg.add_define("UNNI_HOME_ASSISTANT_ENABLED", int(home_assistant_enabled))
    cg.add_define("UNNI_BLE_ENABLED", int(ble_enabled))
    cg.add_define("UNNI_BLE_LIVE_ENABLED", int(config[CONF_BLE_LIVE]))
    cg.add_define("UNNI_BLE_HISTORY_ENABLED", int(config[CONF_BLE_HISTORY]))
    cg.add_define("UNNI_SHT43_IDENTITY_PROBE", int(sht43_identity_probe))
    cg.add_define("RTRH_DEBUG_CAPTURE", int(config[CONF_DEBUG_CAPTURE]))

    if ble_enabled:
        ble = await cg.get_variable(config[CONF_BLE_ID])
        # Set the Sensirion GAP/local name on ESPHome's BLE component itself.
        # This runs before component setup and prevents ESP32BLE from falling
        # back to the ESPHome node name (for example "i2csniffer").
        cg.add(ble.set_name("SHT43 DB" if sht43_identity_probe else "S"))
        esp32_ble.register_gatts_event_handler(ble, var)
        esp32_ble.register_gap_event_handler(ble, var)

        server = await cg.get_variable(config[CONF_BLE_SERVER_ID])
        cg.add(var.set_gatt_server(server))
        cg.add(var.set_ble_advertising_interval(config[CONF_BLE_ADVERTISING_INTERVAL]))
        cg.add(var.set_ble_battery_advertising_interval(config[CONF_BLE_BATTERY_ADVERTISING_INTERVAL]))

    cg.add(var.set_ha_publish_interval(config[CONF_HA_PUBLISH_INTERVAL]))
    cg.add(var.set_sniffer_enabled(config[CONF_SNIFFER_ENABLED]))
    cg.add(var.set_rtrh_enabled(config[CONF_RTRH_ENABLED]))
    cg.add(var.set_rtrh_gpio_setup(config[CONF_RTRH_GPIO_SETUP]))
    cg.add(var.set_rtrh_edge_capture(config[CONF_RTRH_EDGE_CAPTURE]))
    cg.add(var.set_rtrh_decode_only(config[CONF_RTRH_DECODE_ONLY]))
    cg.add(var.set_sniffer_start_delay(config[CONF_SNIFFER_START_DELAY]))
    cg.add(var.set_debug_metrics(config[CONF_DEBUG_METRICS]))
    cg.add(var.set_light_sleep(config[CONF_LIGHT_SLEEP]))
    cg.add(var.set_light_sleep_max_awake(config[CONF_LIGHT_SLEEP_MAX_AWAKE]))
    cg.add(var.set_rtrh_pins(config[CONF_RT_PIN], config[CONF_RH_PIN]))
    cg.add(var.set_co2_pins(config[CONF_CO2_SDA_PIN], config[CONF_CO2_SCL_PIN]))
    cg.add(var.set_battery_pin(config[CONF_BATTERY_PIN]))
    cg.add(var.set_battery_update_interval(config[CONF_BATTERY_UPDATE_INTERVAL]))
    cg.add(var.set_battery_divider_ratio(config[CONF_BATTERY_DIVIDER_RATIO]))
    cg.add(var.set_usb_power_pin(config[CONF_USB_POWER_PIN]))
    cg.add(var.set_energy_save_mode_default(config[CONF_ENERGY_SAVE_MODE_DEFAULT]))
    cg.add(var.set_energy_save_grace(config[CONF_ENERGY_SAVE_GRACE]))

    if home_assistant_enabled:
        energy_save = await switch.new_switch(config[CONF_ENERGY_SAVE_MODE])
        cg.add(energy_save.set_parent(var))
        cg.add(var.set_energy_save_mode_switch(energy_save))
        # A/B diagnostic: the SHT43 identity probe intentionally omits the
        # BLE Pairing Mode ESPHome switch. This keeps all BLE identity, GATT,
        # security and Device Settings code active while restoring the HA
        # switch registry to the known-good single-switch shape.
        if ble_enabled and not sht43_identity_probe and CONF_BLE_PAIRING_MODE in config:
            pairing = await switch.new_switch(config[CONF_BLE_PAIRING_MODE])
            cg.add(pairing.set_parent(var))
            cg.add(var.set_ble_pairing_mode_switch(pairing))
            cg.add(var.set_ble_pairing_window(config[CONF_BLE_PAIRING_WINDOW]))

    cg.add(var.set_thermal_transient_on_rate(config[CONF_THERMAL_TRANSIENT_ON_RATE]))
    cg.add(var.set_thermal_transient_off_rate(config[CONF_THERMAL_TRANSIENT_OFF_RATE]))

    if home_assistant_enabled:
        await _configure_outputs(var, config)
