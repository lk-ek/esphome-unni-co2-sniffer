# SPDX-License-Identifier: GPL-3.0-or-later
import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import binary_sensor, esp32_ble, esp32_ble_server, sensor
from esphome.components.esp32 import add_idf_sdkconfig_option, add_partition
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.core import TimePeriod

# BLE is deliberately NOT a hard dependency. A real `ble: false` build therefore
# does not need esp32_ble / esp32_ble_server in the YAML at all.
DEPENDENCIES = []


def AUTO_LOAD(config):
    loads = ["sensor", "binary_sensor"]
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
CONF_HA_PUBLISH_INTERVAL = "ha_publish_interval"
CONF_SNIFFER_START_DELAY = "sniffer_start_delay"
CONF_DEBUG_METRICS = "debug_metrics"
CONF_DEBUG_CAPTURE = "debug_capture"
CONF_LIGHT_SLEEP = "light_sleep"
CONF_LIGHT_SLEEP_MAX_AWAKE = "light_sleep_max_awake"
CONF_RTRH_G10_PIN = "rtrh_g10_pin"
CONF_RTRH_G13_PIN = "rtrh_g13_pin"
CONF_CO2_SDA_PIN = "co2_sda_pin"
CONF_CO2_SCL_PIN = "co2_scl_pin"
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

bus_sniffer_ns = cg.esphome_ns.namespace("bus_sniffer")
BusSniffer = bus_sniffer_ns.class_("BusSniffer", cg.Component)


def _sensor_schema(**kwargs):
    return sensor.sensor_schema(**kwargs)


def _diagnostic_sensor(**kwargs):
    return _sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC, **kwargs)


def _diagnostic_binary_sensor():
    return binary_sensor.binary_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC)


# YAML key -> (schema, BusSniffer setter)
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
    CONF_THERMAL_TRANSIENT: "set_thermal_transient_sensor",
    CONF_TEMPERATURE_EXTRAPOLATION: "set_temperature_extrapolation_sensor",
    CONF_HUMIDITY_EXTRAPOLATION: "set_humidity_extrapolation_sensor",
    CONF_CALIBRATION_EXTRAPOLATION: "set_calibration_extrapolation_sensor",
}


def _validate_features(config):
    pins = [config[CONF_RTRH_G10_PIN], config[CONF_RTRH_G13_PIN], config[CONF_CO2_SDA_PIN], config[CONF_CO2_SCL_PIN]]
    if len(set(pins)) != len(pins):
        raise cv.Invalid("RT/RH and CO2 GPIOs must be unique")

    if config[CONF_BLE_LIVE] and not config[CONF_BLE]:
        raise cv.Invalid("ble_live: true requires ble: true")
    if config[CONF_BLE_HISTORY] and not config[CONF_BLE]:
        raise cv.Invalid("ble_history: true requires ble: true")

    if not config[CONF_BLE]:
        config.pop(CONF_BLE_ID, None)
        config.pop(CONF_BLE_SERVER_ID, None)

    if not config[CONF_DEBUG_METRICS]:
        for key in DEBUG_SENSOR_DEFAULTS:
            config.pop(key, None)
        for key in DEBUG_BINARY_DEFAULTS:
            config.pop(key, None)

    return config


_SCHEMA = {
    cv.GenerateID(): cv.declare_id(BusSniffer),
    cv.Optional(CONF_BLE, default=True): cv.boolean,
    cv.Optional(CONF_BLE_LIVE, default=True): cv.boolean,
    cv.Optional(CONF_BLE_HISTORY, default=True): cv.boolean,
    cv.GenerateID(CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
    cv.GenerateID(CONF_BLE_SERVER_ID): cv.use_id(esp32_ble_server.BLEServer),
    cv.Optional(CONF_BLE_ADVERTISING_INTERVAL, default="2s"): cv.All(
        cv.positive_time_period_milliseconds,
        cv.Range(min=TimePeriod(milliseconds=20), max=TimePeriod(milliseconds=10240)),
    ),
    cv.Optional(CONF_HA_PUBLISH_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_SNIFFER_START_DELAY, default="0s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_DEBUG_METRICS, default=False): cv.boolean,
    cv.Optional(CONF_DEBUG_CAPTURE, default=False): cv.boolean,
    cv.Optional(CONF_LIGHT_SLEEP, default=True): cv.boolean,
    cv.Optional(CONF_LIGHT_SLEEP_MAX_AWAKE, default="10s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_RTRH_G10_PIN, default=3): cv.int_range(min=0, max=21),
    cv.Optional(CONF_RTRH_G13_PIN, default=4): cv.int_range(min=0, max=21),
    cv.Optional(CONF_CO2_SDA_PIN, default=6): cv.int_range(min=0, max=21),
    cv.Optional(CONF_CO2_SCL_PIN, default=7): cv.int_range(min=0, max=21),
    cv.Optional(CONF_THERMAL_TRANSIENT_ON_RATE, default=0.8): cv.float_range(min=0.05, max=20.0),
    cv.Optional(CONF_THERMAL_TRANSIENT_OFF_RATE, default=0.3): cv.float_range(min=0.01, max=20.0),
}

PRIMARY_SENSOR_DEFAULTS = {
    CONF_CO2: {"name": "CO2", "icon": "mdi:molecule-co2"},
    CONF_RT_TEMPERATURE: {"name": "RT Temperature", "icon": "mdi:thermometer"},
    CONF_RH_HUMIDITY: {"name": "RH Humidity", "icon": "mdi:water-percent"},
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

    cg.add_define("UNNI_BLE_ENABLED", int(ble_enabled))
    cg.add_define("UNNI_BLE_LIVE_ENABLED", int(config[CONF_BLE_LIVE]))
    cg.add_define("UNNI_BLE_HISTORY_ENABLED", int(config[CONF_BLE_HISTORY]))
    cg.add_define("RTRH_DEBUG_CAPTURE", int(config[CONF_DEBUG_CAPTURE]))

    if ble_enabled:
        ble = await cg.get_variable(config[CONF_BLE_ID])
        esp32_ble.register_gatts_event_handler(ble, var)
        esp32_ble.register_gap_event_handler(ble, var)

        server = await cg.get_variable(config[CONF_BLE_SERVER_ID])
        cg.add(var.configure_gatt_server(server))
        cg.add(var.set_ble_advertising_interval(config[CONF_BLE_ADVERTISING_INTERVAL]))

    cg.add(var.set_ha_publish_interval(config[CONF_HA_PUBLISH_INTERVAL]))
    cg.add(var.set_sniffer_start_delay(config[CONF_SNIFFER_START_DELAY]))
    cg.add(var.set_debug_metrics(config[CONF_DEBUG_METRICS]))
    cg.add(var.set_light_sleep(config[CONF_LIGHT_SLEEP]))
    cg.add(var.set_light_sleep_max_awake(config[CONF_LIGHT_SLEEP_MAX_AWAKE]))
    cg.add(var.set_rtrh_pins(config[CONF_RTRH_G10_PIN], config[CONF_RTRH_G13_PIN]))
    cg.add(var.set_co2_pins(config[CONF_CO2_SDA_PIN], config[CONF_CO2_SCL_PIN]))
    cg.add(var.set_thermal_transient_on_rate(config[CONF_THERMAL_TRANSIENT_ON_RATE]))
    cg.add(var.set_thermal_transient_off_rate(config[CONF_THERMAL_TRANSIENT_OFF_RATE]))

    await _configure_outputs(var, config)
