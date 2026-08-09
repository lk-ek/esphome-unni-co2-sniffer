import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import sensor
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

DEPENDENCIES = ["web_server"]
AUTO_LOAD = ["sensor"]

CONF_CO2 = "co2"
CONF_CRC_ERRORS = "crc_errors"
CONF_FRAME_ERRORS = "frame_errors"
CONF_RT_TEMPERATURE = "rt_temperature"
CONF_RH_HUMIDITY = "rh_humidity"

bus_sniffer_ns = cg.esphome_ns.namespace("bus_sniffer")
BusSniffer = bus_sniffer_ns.class_("BusSniffer", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BusSniffer),

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
