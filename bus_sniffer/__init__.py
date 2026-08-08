import esphome.codegen as cg
import esphome.config_validation as cv

DEPENDENCIES = ["web_server"]

bus_sniffer_ns = cg.esphome_ns.namespace("bus_sniffer")

BusSniffer = bus_sniffer_ns.class_(
    "BusSniffer",
    cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {}
).extend(
    cv.COMPONENT_SCHEMA
)


async def to_code(config):
    var = cg.new_Pvariable(
        BusSniffer
    )

    await cg.register_component(
        var,
        config
    )

