import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_BQ769X0_ID, BQ769X0Component

DEPENDENCIES = ["bq769x0"]

CONF_ALARMS = "alarms"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BQ769X0_ID): cv.use_id(BQ769X0Component),
        # Comma-separated list of active alarms, or "OK".
        cv.Optional(CONF_ALARMS): text_sensor.text_sensor_schema(
            icon="mdi:alert-circle-outline"
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_BQ769X0_ID])
    if alarms_config := config.get(CONF_ALARMS):
        sens = await text_sensor.new_text_sensor(alarms_config)
        cg.add(hub.set_alarms_sensor(sens))
