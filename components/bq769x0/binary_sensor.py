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
CONF_CHARGE_FET = "charge_fet"
CONF_DISCHARGE_FET = "discharge_fet"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BQ769X0_ID): cv.use_id(BQ769X0Component),
        # State of the CHG and DSG FET drivers as reported by the device.
        cv.Optional(CONF_CHARGE_FET): binary_sensor.binary_sensor_schema(
            icon="mdi:battery-charging"
        ),
        cv.Optional(CONF_DISCHARGE_FET): binary_sensor.binary_sensor_schema(
            icon="mdi:battery-arrow-down"
        ),
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
    if charge_config := config.get(CONF_CHARGE_FET):
        sens = await binary_sensor.new_binary_sensor(charge_config)
        cg.add(hub.set_charge_fet_sensor(sens))
    if discharge_config := config.get(CONF_DISCHARGE_FET):
        sens = await binary_sensor.new_binary_sensor(discharge_config)
        cg.add(hub.set_discharge_fet_sensor(sens))
    for cell in range(1, MAX_CELLS + 1):
        if cell_config := config.get(cell_key(BALANCING_PREFIX, cell)):
            sens = await binary_sensor.new_binary_sensor(cell_config)
            cg.add(hub.set_balancing_sensor(cell - 1, sens))
