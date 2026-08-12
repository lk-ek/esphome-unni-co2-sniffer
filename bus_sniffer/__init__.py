# SPDX-License-Identifier: GPL-3.0-or-later
import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import binary_sensor, esp32_ble, esp32_ble_server, sensor
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.core import TimePeriod

# BLE is deliberately NOT a hard dependency. A real `ble: false` build therefore
# does not need esp32_ble / esp32_ble_server in the YAML at all.
DEPENDENCIES = []
AUTO_LOAD = ["sensor", "binary_sensor"]

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
    if config[CONF_BLE_LIVE] and not config[CONF_BLE]:
        raise cv.Invalid("ble_live: true requires ble: true")
    if config[CONF_BLE_HISTORY] and not config[CONF_BLE]:
        raise cv.Invalid("ble_history: true requires ble: true")

    if config[CONF_BLE]:
        if CONF_BLE_ID not in config:
            raise cv.Invalid("ble: true requires ble_id")
        if CONF_BLE_SERVER_ID not in config:
            raise cv.Invalid("ble: true requires ble_server_id")

    return config


_SCHEMA = {
    cv.GenerateID(): cv.declare_id(BusSniffer),
    cv.Optional(CONF_BLE, default=True): cv.boolean,
    cv.Optional(CONF_BLE_LIVE, default=True): cv.boolean,
    cv.Optional(CONF_BLE_HISTORY, default=True): cv.boolean,
    cv.Optional(CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
    cv.Optional(CONF_BLE_SERVER_ID): cv.use_id(esp32_ble_server.BLEServer),
    cv.Optional(CONF_BLE_ADVERTISING_INTERVAL, default="2s"): cv.All(
        cv.positive_time_period_milliseconds,
        cv.Range(min=TimePeriod(milliseconds=20), max=TimePeriod(milliseconds=10240)),
    ),
    cv.Optional(CONF_HA_PUBLISH_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_SNIFFER_START_DELAY, default="0s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_DEBUG_METRICS, default=False): cv.boolean,
    cv.Optional(CONF_DEBUG_CAPTURE, default=False): cv.boolean,
    cv.Optional(CONF_THERMAL_TRANSIENT_ON_RATE, default=0.8): cv.float_range(min=0.05, max=20.0),
    cv.Optional(CONF_THERMAL_TRANSIENT_OFF_RATE, default=0.3): cv.float_range(min=0.01, max=20.0),
}

for key, (schema, _) in SENSOR_OUTPUTS.items():
    _SCHEMA[cv.Required(key) if key == CONF_CO2 else cv.Optional(key)] = schema
for key in BINARY_OUTPUTS:
    _SCHEMA[cv.Optional(key)] = _diagnostic_binary_sensor()

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
    cg.add(var.set_thermal_transient_on_rate(config[CONF_THERMAL_TRANSIENT_ON_RATE]))
    cg.add(var.set_thermal_transient_off_rate(config[CONF_THERMAL_TRANSIENT_OFF_RATE]))

    await _configure_outputs(var, config)
