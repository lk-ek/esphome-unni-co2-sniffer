import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import sensor, esp32_ble, esp32_ble_server
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.core import TimePeriod

# BLE is deliberately NOT a hard dependency anymore.  A real `ble: false`
# build therefore does not need esp32_ble / esp32_ble_server in the YAML at all.
DEPENDENCIES = ["web_server"]
AUTO_LOAD = ["sensor"]

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

            cv.Optional(CONF_BLE_ADVERTISING_INTERVAL, default="2s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=TimePeriod(milliseconds=20),
                    max=TimePeriod(milliseconds=10240),
                ),
            ),
            cv.Optional(CONF_HA_PUBLISH_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,

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
