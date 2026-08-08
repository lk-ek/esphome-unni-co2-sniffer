import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["web_server"]
AUTO_LOAD = ["sensor"]

CONF_CO2 = "co2"

bus_sniffer_ns = cg.esphome_ns.namespace("bus_sniffer")

BusSniffer = bus_sniffer_ns.class_(
    "BusSniffer",
    cg.Component,
)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BusSniffer),

        cv.Required(CONF_CO2): sensor.sensor_schema(
            unit_of_measurement="ppm",
            accuracy_decimals=0,
            device_class="carbon_dioxide",
            state_class="measurement",
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(
        var,
        config,
    )

    co2 = await sensor.new_sensor(
        config[CONF_CO2]
    )

    cg.add(
        var.set_co2_sensor(co2)
    )
