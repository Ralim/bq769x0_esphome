import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import sensor
from esphome.const import (
    CONF_CURRENT,
    CONF_SHUNT_RESISTANCE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_VOLT,
)

from . import (
    CONF_BQ769X0_ID,
    MAX_CELLS,
    BQ769X0Component,
    cell_key,
    validate_cells_within_count,
)

DEPENDENCIES = ["bq769x0"]

CONF_PACK_VOLTAGE = "pack_voltage"
CELL_PREFIX = "cell_"

_VOLTAGE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_VOLT,
    accuracy_decimals=3,
    device_class=DEVICE_CLASS_VOLTAGE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BQ769X0_ID): cv.use_id(BQ769X0Component),
        cv.Optional(CONF_PACK_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        **{
            cv.Optional(cell_key(CELL_PREFIX, cell)): _VOLTAGE_SCHEMA
            for cell in range(1, MAX_CELLS + 1)
        },
    }
)


def _final_validate(config):
    validate_cells_within_count(CELL_PREFIX)(config)
    if CONF_CURRENT in config:
        fv.id_declaration_match_schema(
            {
                cv.Required(
                    CONF_SHUNT_RESISTANCE,
                    msg=f"{CONF_SHUNT_RESISTANCE} is required to measure current",
                ): cv.valid
            }
        )(config[CONF_BQ769X0_ID])
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BQ769X0_ID])
    if pack_config := config.get(CONF_PACK_VOLTAGE):
        cg.add(hub.set_pack_voltage_sensor(await sensor.new_sensor(pack_config)))
    if current_config := config.get(CONF_CURRENT):
        cg.add(hub.set_current_sensor(await sensor.new_sensor(current_config)))
    for cell in range(1, MAX_CELLS + 1):
        if cell_config := config.get(cell_key(CELL_PREFIX, cell)):
            sens = await sensor.new_sensor(cell_config)
            cg.add(hub.set_cell_voltage_sensor(cell - 1, sens))
