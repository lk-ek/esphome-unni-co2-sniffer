import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import sensor, esp32_ble, esp32_ble_server
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.core import TimePeriod

DEPENDENCIES = ["web_server", "esp32_ble", "esp32_ble_server"]
AUTO_LOAD = ["sensor"]

CONF_CO2 = "co2"
CONF_CRC_ERRORS = "crc_errors"
CONF_FRAME_ERRORS = "frame_errors"
CONF_RT_TEMPERATURE = "rt_temperature"
CONF_RH_HUMIDITY = "rh_humidity"
CONF_BLE_SERVER_ID = "ble_server_id"
CONF_BLE_ADVERTISING_INTERVAL = "ble_advertising_interval"
CONF_HA_PUBLISH_INTERVAL = "ha_publish_interval"

bus_sniffer_ns = cg.esphome_ns.namespace("bus_sniffer")
BusSniffer = bus_sniffer_ns.class_("BusSniffer", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BusSniffer),
        cv.GenerateID(esp32_ble.CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
        cv.GenerateID(CONF_BLE_SERVER_ID): cv.use_id(esp32_ble_server.BLEServer),
        cv.Optional(CONF_BLE_ADVERTISING_INTERVAL, default="2s"): cv.All(
            cv.positive_time_period_milliseconds, cv.Range(min=TimePeriod(milliseconds=20), max=TimePeriod(milliseconds=10240))
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
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    ble = await cg.get_variable(config[esp32_ble.CONF_BLE_ID])
    esp32_ble.register_gatts_event_handler(ble, var)
    esp32_ble.register_gap_event_handler(ble, var)

    # GATT topology is owned by the component, not by user YAML.  This call is
    # emitted during generated setup, before App.setup(), so the services and
    # characteristics exist before the ESPHome BLE server starts registering.
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
