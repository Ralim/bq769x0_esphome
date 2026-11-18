import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import (
    CONF_BQ769X0_ID,
    MAX_CELLS,
    BQ769X0Component,
    cell_key,
    validate_cells_within_count,
)

DEPENDENCIES = ["bq769x0"]

BALANCING_PREFIX = "balancing_cell_"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BQ769X0_ID): cv.use_id(BQ769X0Component),
        **{
            cv.Optional(
                cell_key(BALANCING_PREFIX, cell)
            ): binary_sensor.binary_sensor_schema(icon="mdi:scale-balance")
            for cell in range(1, MAX_CELLS + 1)
        },
    }
)

FINAL_VALIDATE_SCHEMA = validate_cells_within_count(BALANCING_PREFIX)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BQ769X0_ID])
    for cell in range(1, MAX_CELLS + 1):
        if cell_config := config.get(cell_key(BALANCING_PREFIX, cell)):
            sens = await binary_sensor.new_binary_sensor(cell_config)
            cg.add(hub.set_balancing_sensor(cell - 1, sens))
