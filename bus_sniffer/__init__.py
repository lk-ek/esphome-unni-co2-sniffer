import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import sensor, binary_sensor, esp32_ble, esp32_ble_server
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.core import TimePeriod

# BLE is deliberately NOT a hard dependency anymore.  A real `ble: false`
# build therefore does not need esp32_ble / esp32_ble_server in the YAML at all.
DEPENDENCIES = ["web_server"]
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


def _validate_features(config):
    ble = config[CONF_BLE]
    live = config[CONF_BLE_LIVE]
    history = config[CONF_BLE_HISTORY]

    if live and not ble:
        raise cv.Invalid("ble_live: true requires ble: true")
    if history and not ble:
        raise cv.Invalid("ble_history: true requires ble: true")

    if ble:
        if CONF_BLE_ID not in config:
            raise cv.Invalid("ble: true requires ble_id")
        if CONF_BLE_SERVER_ID not in config:
            raise cv.Invalid("ble: true requires ble_server_id")

    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BusSniffer),

            # Feature switches. Defaults preserve the current full-BLE build.
            cv.Optional(CONF_BLE, default=True): cv.boolean,
            cv.Optional(CONF_BLE_LIVE, default=True): cv.boolean,
            cv.Optional(CONF_BLE_HISTORY, default=True): cv.boolean,

            # Explicit references are needed only for ble:true builds.  Making
            # them optional is what permits a genuine no-BLE YAML/build.
            cv.Optional(CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
            cv.Optional(CONF_BLE_SERVER_ID): cv.use_id(esp32_ble_server.BLEServer),

            cv.Optional(CONF_BLE_ADVERTISING_INTERVAL, default="10s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=TimePeriod(milliseconds=20),
                    max=TimePeriod(milliseconds=10240),
                ),
            ),
            cv.Optional(CONF_HA_PUBLISH_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_SNIFFER_START_DELAY, default="0s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DEBUG_METRICS, default=False): cv.boolean,
            cv.Optional(CONF_THERMAL_TRANSIENT_ON_RATE, default=0.8): cv.float_range(
                min=0.05, max=20.0
            ),
            cv.Optional(CONF_THERMAL_TRANSIENT_OFF_RATE, default=0.3): cv.float_range(
                min=0.01, max=20.0
            ),

            cv.Optional(CONF_REF_PERIOD): sensor.sensor_schema(
                unit_of_measurement="µs",
                accuracy_decimals=3,
                state_class="measurement",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RT_PERIOD): sensor.sensor_schema(
                unit_of_measurement="µs",
                accuracy_decimals=3,
                state_class="measurement",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RH_STATE_PERIOD): sensor.sensor_schema(
                unit_of_measurement="µs",
                accuracy_decimals=1,
                state_class="measurement",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RT_RATIO): sensor.sensor_schema(
                accuracy_decimals=6,
                state_class="measurement",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RH_RATIO): sensor.sensor_schema(
                accuracy_decimals=6,
                state_class="measurement",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RH_LOG): sensor.sensor_schema(
                accuracy_decimals=6,
                state_class="measurement",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_MEASUREMENT_QUALITY): sensor.sensor_schema(
                unit_of_measurement="%",
                accuracy_decimals=0,
                state_class="measurement",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_THERMAL_TRANSIENT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_TEMPERATURE_EXTRAPOLATION): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_HUMIDITY_EXTRAPOLATION): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_CALIBRATION_EXTRAPOLATION): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),

            cv.Required(CONF_CO2): sensor.sensor_schema(
                unit_of_measurement="ppm",
                accuracy_decimals=0,
                device_class="carbon_dioxide",
                state_class="measurement",
            ),
            cv.Optional(CONF_CRC_ERRORS): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class="total_increasing",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_FRAME_ERRORS): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class="total_increasing",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RT_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement="°C",
                accuracy_decimals=1,
                device_class="temperature",
                state_class="measurement",
            ),
            cv.Optional(CONF_RH_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement="%",
                accuracy_decimals=1,
                device_class="humidity",
                state_class="measurement",
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_features,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    ble_enabled = config[CONF_BLE]
    ble_live_enabled = config[CONF_BLE_LIVE]
    ble_history_enabled = config[CONF_BLE_HISTORY]

    cg.add_define("UNNI_BLE_ENABLED", 1 if ble_enabled else 0)
    cg.add_define("UNNI_BLE_LIVE_ENABLED", 1 if ble_live_enabled else 0)
    cg.add_define("UNNI_BLE_HISTORY_ENABLED", 1 if ble_history_enabled else 0)

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

    if CONF_REF_PERIOD in config:
        s = await sensor.new_sensor(config[CONF_REF_PERIOD])
        cg.add(var.set_ref_period_sensor(s))
    if CONF_RT_PERIOD in config:
        s = await sensor.new_sensor(config[CONF_RT_PERIOD])
        cg.add(var.set_rt_period_sensor(s))
    if CONF_RH_STATE_PERIOD in config:
        s = await sensor.new_sensor(config[CONF_RH_STATE_PERIOD])
        cg.add(var.set_rh_state_period_sensor(s))
    if CONF_RT_RATIO in config:
        s = await sensor.new_sensor(config[CONF_RT_RATIO])
        cg.add(var.set_rt_ratio_sensor(s))
    if CONF_RH_RATIO in config:
        s = await sensor.new_sensor(config[CONF_RH_RATIO])
        cg.add(var.set_rh_ratio_sensor(s))
    if CONF_RH_LOG in config:
        s = await sensor.new_sensor(config[CONF_RH_LOG])
        cg.add(var.set_rh_log_sensor(s))
    if CONF_MEASUREMENT_QUALITY in config:
        s = await sensor.new_sensor(config[CONF_MEASUREMENT_QUALITY])
        cg.add(var.set_measurement_quality_sensor(s))
    if CONF_THERMAL_TRANSIENT in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_THERMAL_TRANSIENT])
        cg.add(var.set_thermal_transient_sensor(s))
    if CONF_TEMPERATURE_EXTRAPOLATION in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_TEMPERATURE_EXTRAPOLATION])
        cg.add(var.set_temperature_extrapolation_sensor(s))
    if CONF_HUMIDITY_EXTRAPOLATION in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_HUMIDITY_EXTRAPOLATION])
        cg.add(var.set_humidity_extrapolation_sensor(s))
    if CONF_CALIBRATION_EXTRAPOLATION in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_CALIBRATION_EXTRAPOLATION])
        cg.add(var.set_calibration_extrapolation_sensor(s))

    co2 = await sensor.new_sensor(config[CONF_CO2])
    cg.add(var.set_co2_sensor(co2))

    if CONF_CRC_ERRORS in config:
        s = await sensor.new_sensor(config[CONF_CRC_ERRORS])
        cg.add(var.set_crc_errors_sensor(s))

    if CONF_FRAME_ERRORS in config:
        s = await sensor.new_sensor(config[CONF_FRAME_ERRORS])
        cg.add(var.set_frame_errors_sensor(s))

    if CONF_RT_TEMPERATURE in config:
        s = await sensor.new_sensor(config[CONF_RT_TEMPERATURE])
        cg.add(var.set_rt_temperature_sensor(s))

    if CONF_RH_HUMIDITY in config:
        s = await sensor.new_sensor(config[CONF_RH_HUMIDITY])
        cg.add(var.set_rh_humidity_sensor(s))
