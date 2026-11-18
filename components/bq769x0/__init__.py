import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome import pins
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_MODEL,
    CONF_SHUNT_RESISTANCE,
    CONF_WAKEUP_PIN,
)

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

CONF_BQ769X0_ID = "bq769x0_id"
CONF_ALERT_PIN = "alert_pin"
CONF_BALANCING = "balancing"
CONF_CELL_COUNT = "cell_count"
CONF_CRC = "crc"
CONF_ENABLED = "enabled"
CONF_MINIMUM_DELTA = "minimum_delta"
CONF_MINIMUM_VOLTAGE = "minimum_voltage"
CONF_MINIMUM_CELL_VOLTAGE = "minimum_cell_voltage"
CONF_MAXIMUM_CELL_VOLTAGE = "maximum_cell_voltage"
CONF_MAXIMUM_CHARGE_CURRENT = "maximum_charge_current"
CONF_MAXIMUM_DISCHARGE_CURRENT = "maximum_discharge_current"
CONF_OVERCURRENT_RECOVERY_TIME = "overcurrent_recovery_time"
CONF_SHORT_CIRCUIT_CURRENT = "short_circuit_current"
CONF_SHORT_CIRCUIT_DELAY = "short_circuit_delay"
CONF_DISCHARGE_OVERCURRENT_DELAY = "discharge_overcurrent_delay"
CONF_OVERVOLTAGE_DELAY = "overvoltage_delay"
CONF_UNDERVOLTAGE_DELAY = "undervoltage_delay"

# Supported hardware protection delays, in register code order.
SHORT_CIRCUIT_DELAYS = ["70us", "100us", "200us", "400us"]
DISCHARGE_OVERCURRENT_DELAYS = [
    "8ms",
    "20ms",
    "40ms",
    "80ms",
    "160ms",
    "320ms",
    "640ms",
    "1280ms",
]
OVERVOLTAGE_DELAYS = ["1s", "2s", "4s", "8s"]
UNDERVOLTAGE_DELAYS = ["1s", "4s", "8s", "16s"]
# Largest hardware thresholds across the sense resistor (upper range).
MAX_DISCHARGE_OVERCURRENT_MV = 100
MAX_SHORT_CIRCUIT_MV = 200

MAX_CELLS = 15

bq769x0_ns = cg.esphome_ns.namespace("bq769x0")
BQ769X0Component = bq769x0_ns.class_(
    "BQ769X0Component", cg.PollingComponent, i2c.I2CDevice
)

Model = bq769x0_ns.enum("Model", is_class=True)
MODELS = {
    "BQ76920": Model.BQ76920,
    "BQ76930": Model.BQ76930,
    "BQ76940": Model.BQ76940,
}
# Supported (min, max) cell count per model.
MODEL_CELL_RANGE = {
    "BQ76920": (3, 5),
    "BQ76930": (6, 10),
    "BQ76940": (9, 15),
}


def _validate_cell_count(config):
    low, high = MODEL_CELL_RANGE[config[CONF_MODEL]]
    if not low <= config[CONF_CELL_COUNT] <= high:
        raise cv.Invalid(
            f"{config[CONF_MODEL]} supports {low} to {high} cells",
            path=[CONF_CELL_COUNT],
        )
    return config


def _validate_cell_voltage_limits(config):
    low = config.get(CONF_MINIMUM_CELL_VOLTAGE)
    high = config.get(CONF_MAXIMUM_CELL_VOLTAGE)
    if low is not None and high is not None and low >= high:
        raise cv.Invalid(
            f"{CONF_MINIMUM_CELL_VOLTAGE} must be below {CONF_MAXIMUM_CELL_VOLTAGE}",
            path=[CONF_MINIMUM_CELL_VOLTAGE],
        )
    return config


def _validate_current_limits(config):
    for key in (
        CONF_MAXIMUM_CHARGE_CURRENT,
        CONF_MAXIMUM_DISCHARGE_CURRENT,
        CONF_SHORT_CIRCUIT_CURRENT,
    ):
        if key in config and CONF_SHUNT_RESISTANCE not in config:
            raise cv.Invalid(
                f"{CONF_SHUNT_RESISTANCE} is required to limit current", path=[key]
            )
    shunt = config.get(CONF_SHUNT_RESISTANCE)
    for key, max_mv in (
        (CONF_MAXIMUM_DISCHARGE_CURRENT, MAX_DISCHARGE_OVERCURRENT_MV),
        (CONF_SHORT_CIRCUIT_CURRENT, MAX_SHORT_CIRCUIT_MV),
    ):
        if key in config and config[key] * shunt * 1000 > max_mv:
            raise cv.Invalid(
                f"Above the hardware limit of {max_mv / (shunt * 1000):.2f} A "
                f"({max_mv} mV across {CONF_SHUNT_RESISTANCE})",
                path=[key],
            )
    if (
        CONF_SHORT_CIRCUIT_CURRENT in config
        and CONF_MAXIMUM_DISCHARGE_CURRENT in config
        and config[CONF_SHORT_CIRCUIT_CURRENT] <= config[CONF_MAXIMUM_DISCHARGE_CURRENT]
    ):
        raise cv.Invalid(
            f"{CONF_SHORT_CIRCUIT_CURRENT} must be above {CONF_MAXIMUM_DISCHARGE_CURRENT}",
            path=[CONF_SHORT_CIRCUIT_CURRENT],
        )
    return config


def _delay_code(options):
    """Validate a time that the hardware supports, returning its register code."""
    allowed = [
        cv.positive_time_period_microseconds(option).total_microseconds
        for option in options
    ]

    def validator(value):
        us = cv.positive_time_period_microseconds(value).total_microseconds
        if us not in allowed:
            raise cv.Invalid(f"Must be one of {', '.join(options)}")
        return allowed.index(us)

    return validator


_POSITIVE_VOLTAGE = cv.All(cv.voltage, cv.Range(min=0.0, min_included=False))
_POSITIVE_CURRENT = cv.All(cv.current, cv.Range(min=0.0, min_included=False))

# Odd-even balancing: every update, the cells that are above `minimum_voltage`
# and more than `minimum_delta` above the lowest cell are bled down, alternating
# between odd and even cells so that adjacent cells are never balanced together.
BALANCING_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ENABLED, default=True): cv.boolean,
        cv.Optional(CONF_MINIMUM_DELTA, default="10mV"): _POSITIVE_VOLTAGE,
        cv.Optional(CONF_MINIMUM_VOLTAGE, default="3450mV"): _POSITIVE_VOLTAGE,
    }
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BQ769X0Component),
            cv.Required(CONF_MODEL): cv.one_of(*MODELS, upper=True),
            cv.Required(CONF_CELL_COUNT): cv.int_range(min=1, max=MAX_CELLS),
            # Whether CRC is used depends on the orderable part number (see the
            # datasheet device comparison table), so it must be set explicitly.
            cv.Required(CONF_CRC): cv.boolean,
            # Required for current measurement, enables the coulomb counter.
            cv.Optional(CONF_SHUNT_RESISTANCE): cv.All(
                cv.resistance, cv.Range(min=0.0, min_included=False)
            ),
            cv.Optional(CONF_ALERT_PIN): pins.gpio_input_pin_schema,
            # Pulses TS1 high at boot to bring the device out of SHIP mode.
            cv.Optional(CONF_WAKEUP_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_BALANCING): BALANCING_SCHEMA,
            # Discharge FET is turned off while any cell is below this.
            cv.Optional(CONF_MINIMUM_CELL_VOLTAGE): _POSITIVE_VOLTAGE,
            # Charge FET is turned off while any cell is above this.
            cv.Optional(CONF_MAXIMUM_CELL_VOLTAGE): _POSITIVE_VOLTAGE,
            # Charge/discharge FET is turned off while the current exceeds this.
            cv.Optional(CONF_MAXIMUM_CHARGE_CURRENT): _POSITIVE_CURRENT,
            cv.Optional(CONF_MAXIMUM_DISCHARGE_CURRENT): _POSITIVE_CURRENT,
            # How long a FET stays off after an overcurrent trip before it is retried.
            cv.Optional(
                CONF_OVERCURRENT_RECOVERY_TIME, default="3s"
            ): cv.positive_time_period_milliseconds,
            # Hardware short-circuit protection: both FETs off until the load is removed.
            cv.Optional(CONF_SHORT_CIRCUIT_CURRENT): _POSITIVE_CURRENT,
            # Hardware protection delays; the defaults match the device's power-on defaults.
            cv.Optional(CONF_SHORT_CIRCUIT_DELAY, default="70us"): _delay_code(
                SHORT_CIRCUIT_DELAYS
            ),
            cv.Optional(CONF_DISCHARGE_OVERCURRENT_DELAY, default="8ms"): _delay_code(
                DISCHARGE_OVERCURRENT_DELAYS
            ),
            cv.Optional(CONF_OVERVOLTAGE_DELAY, default="1s"): _delay_code(
                OVERVOLTAGE_DELAYS
            ),
            cv.Optional(CONF_UNDERVOLTAGE_DELAY, default="1s"): _delay_code(
                UNDERVOLTAGE_DELAYS
            ),
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(i2c.i2c_device_schema(0x08)),
    _validate_cell_count,
    _validate_cell_voltage_limits,
    _validate_current_limits,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_model(MODELS[config[CONF_MODEL]]))
    cg.add(var.set_cell_count(config[CONF_CELL_COUNT]))
    cg.add(var.set_crc_enabled(config[CONF_CRC]))
    if CONF_SHUNT_RESISTANCE in config:
        cg.add(var.set_shunt_resistance(config[CONF_SHUNT_RESISTANCE]))
    if CONF_ALERT_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_ALERT_PIN])
        cg.add(var.set_alert_pin(pin))
    if CONF_WAKEUP_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_WAKEUP_PIN])
        cg.add(var.set_wakeup_pin(pin))
    if balancing := config.get(CONF_BALANCING):
        cg.add(var.set_balancing_enabled(balancing[CONF_ENABLED]))
        cg.add(var.set_balancing_minimum_delta(balancing[CONF_MINIMUM_DELTA]))
        cg.add(var.set_balancing_minimum_voltage(balancing[CONF_MINIMUM_VOLTAGE]))
    if CONF_MINIMUM_CELL_VOLTAGE in config:
        cg.add(var.set_minimum_cell_voltage(config[CONF_MINIMUM_CELL_VOLTAGE]))
    if CONF_MAXIMUM_CELL_VOLTAGE in config:
        cg.add(var.set_maximum_cell_voltage(config[CONF_MAXIMUM_CELL_VOLTAGE]))
    if CONF_MAXIMUM_CHARGE_CURRENT in config:
        cg.add(var.set_maximum_charge_current(config[CONF_MAXIMUM_CHARGE_CURRENT]))
    if CONF_MAXIMUM_DISCHARGE_CURRENT in config:
        cg.add(
            var.set_maximum_discharge_current(config[CONF_MAXIMUM_DISCHARGE_CURRENT])
        )
    cg.add(var.set_overcurrent_recovery_time(config[CONF_OVERCURRENT_RECOVERY_TIME]))
    if CONF_SHORT_CIRCUIT_CURRENT in config:
        cg.add(var.set_short_circuit_current(config[CONF_SHORT_CIRCUIT_CURRENT]))
    cg.add(var.set_short_circuit_delay_code(config[CONF_SHORT_CIRCUIT_DELAY]))
    cg.add(
        var.set_discharge_overcurrent_delay_code(
            config[CONF_DISCHARGE_OVERCURRENT_DELAY]
        )
    )
    cg.add(var.set_overvoltage_delay_code(config[CONF_OVERVOLTAGE_DELAY]))
    cg.add(var.set_undervoltage_delay_code(config[CONF_UNDERVOLTAGE_DELAY]))


def cell_key(prefix, cell):
    """Config key for the one-based `cell`."""
    return f"{prefix}{cell}"


def validate_cells_within_count(prefix):
    """Final validator rejecting per-cell keys above the hub's cell_count."""

    def validator(config):
        full_config = fv.full_config.get()
        hub_path = full_config.get_path_for_id(config[CONF_BQ769X0_ID])[:-1]
        count = full_config.get_config_for_path(hub_path)[CONF_CELL_COUNT]
        for cell in range(count + 1, MAX_CELLS + 1):
            if cell_key(prefix, cell) in config:
                raise cv.Invalid(
                    f"Cell {cell} is above {CONF_CELL_COUNT} ({count})",
                    path=[cell_key(prefix, cell)],
                )
        return config

    return validator
