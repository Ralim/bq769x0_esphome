# BQ769x0 driver for ESPHome

External ESPHome component for the TI bq76920 / bq76930 / bq76940 battery monitor AFEs (I2C).

## Usage

```yaml
external_components:
  - source: github://Ralim/bq769x0_esphome
    components: [bq769x0]

i2c:
  sda: GPIO6
  scl: GPIO5

bq769x0:
  id: bms
  model: BQ76920 # BQ76920, BQ76930 or BQ76940
  cell_count: 4 # cells connected; a different detected count is reported as a warning
  crc: false # depends on the orderable part number, see the datasheet
  address: 0x18 # default 0x08, some part numbers use 0x18
  shunt_resistance: 1mOhm # enables the coulomb counter, required for the current sensor
  update_interval: 5s
  alert_pin: GPIO10 # optional, SYS_STAT is read and cleared when asserted
  wakeup_pin: GPIO4 # optional, pulses TS1 at boot to leave SHIP mode
  minimum_cell_voltage: 2650mV # optional, discharge FET off while any cell is below this
  maximum_cell_voltage: 3550mV # optional, charge FET off while any cell is above this
  maximum_charge_current: 10A # optional, charge FET off while charging faster than this
  maximum_discharge_current: 20A # optional, discharge FET off while discharging faster than this
  overcurrent_recovery_time: 3s # default 3s, how long a FET stays off after an overcurrent before retrying
  balancing: # optional, balancing is left off when omitted
    enabled: true
    minimum_delta: 10mV # a cell must be more than this above the lowest cell
    minimum_voltage: 3450mV # and above this voltage to be balanced down

sensor:
  - platform: bq769x0
    bq769x0_id: bms
    pack_voltage:
      name: Pack Voltage
    current:
      name: Pack Current
    max_cell_delta: # highest minus lowest cell, in mV
      name: Max Cell Delta
    cell_1: # cell_1 .. cell_<cell_count>
      name: Cell 1 Voltage

binary_sensor:
  - platform: bq769x0
    bq769x0_id: bms
    charge_fet: # CHG FET driver state
      name: Charge FET
    discharge_fet: # DSG FET driver state
      name: Discharge FET
    balancing_cell_1: # balancing_cell_1 .. balancing_cell_<cell_count>
      name: Cell 1 Balancing

text_sensor:
  - platform: bq769x0
    bq769x0_id: bms
    alarms: # active alarms separated by ", ", or "OK"
      name: Alarms
```

The alarms are `Over Cell Voltage` / `Under Cell Voltage` (a cell outside `maximum_cell_voltage` /
`minimum_cell_voltage`), `Over Charge Current` / `Over Discharge Current` (while the FET is held off before its
retry) and `Short Circuit` (the device's short-circuit protection tripped, held until the load is removed).

Cells are numbered from the bottom of the stack, skipping cell inputs that are shorted (unused), so on a
bq76920 wired for 4S with VC4 shorted, `cell_4` reads VC5.

Balancing uses the classic odd-even scheme: on each update the cells needing balancing are
found, then only the odd cells (2nd, 4th, ...) or the even cells are balanced, alternating each update, so two
adjacent cells are never balanced at once. If one half has nothing to balance the other half runs instead.
Balancing is turned off whenever the cell readings are invalid or the spread is implausibly large.

When `minimum_cell_voltage` and/or `maximum_cell_voltage` are set, the corresponding FET is switched on every
update while all cells are within the limit and off otherwise. Both controlled FETs are turned off while the cell
readings are invalid. After a short circuit both FETs stay off until the device reports the load has been
removed. FETs without a configured limit are left untouched.

The current limits are checked on every fresh coulomb counter reading (every 250 ms when `alert_pin` is set,
otherwise every update). Exceeding one turns that FET off and automatically retries it
`overcurrent_recovery_time` (default 3 s) after the trip; if the current is still too high it trips again on the
next reading.

### Hardware protection

The device's own protections are programmed from the same settings at every boot, so they keep working even if
the ESP stops responding. Each threshold is rounded to the nearest step the hardware supports that is at or beyond
the configured value, so the software limits normally act first and the hardware is the backstop. The resulting
thresholds are shown in the config log.

| Hardware protection              | Threshold from              | Delay setting (supported values)                                      | On trip                                                  |
| -------------------------------- | --------------------------- | --------------------------------------------------------------------- | -------------------------------------------------------- |
| Overvoltage (OV)                 | `maximum_cell_voltage`      | `overvoltage_delay` (1, 2, 4, 8 s)                                    | charge off                                               |
| Undervoltage (UV)                | `minimum_cell_voltage`      | `undervoltage_delay` (1, 4, 8, 16 s)                                  | discharge off                                            |
| Discharge overcurrent (OCD)      | `maximum_discharge_current` | `discharge_overcurrent_delay` (8, 20, 40, 80, 160, 320, 640, 1280 ms) | discharge off, retried after `overcurrent_recovery_time` |
| Short circuit in discharge (SCD) | `short_circuit_current`     | `short_circuit_delay` (70, 100, 200, 400 us)                          | both off until the load is removed                       |

```yaml
bq769x0:
  short_circuit_current: 40A # optional, must be above maximum_discharge_current
  short_circuit_delay: 70us # default 70us
  discharge_overcurrent_delay: 8ms # default 8ms
  overvoltage_delay: 1s # default 1s
  undervoltage_delay: 1s # default 1s
```

- The delays default to the device's power-on values.
- OCD and SCD are measured across the sense resistor: 8-50 mV / 22-100 mV, or 17-100 mV / 44-200 mV when either
  needs the upper range (the range is shared). A current above the upper range is a config error.
- A protection without its setting keeps the lowest threshold of the selected range (for OV/UV, the device
  default). Without `maximum_discharge_current` a hardware OCD trip is handled like a short circuit.
- The OV/UV ranges depend on each device's calibration (roughly 3.15-4.7 V and 1.58-3.1 V); a limit outside them
  is clamped and logged as a warning.
- There is no hardware charge overcurrent protection; `maximum_charge_current` is enforced in software only.

See `bq769x0_demo.yaml` for a complete configuration. Its parameters can be overridden on the command line,
e.g. `esphome -s cell_count 5 run bq769x0_demo.yaml`.

## Development

```sh
uv venv .venv && uv pip install --python .venv esphome yamllint ruff==0.17.5 clang-format==23.1.5
.venv/bin/esphome compile bq769x0_demo.yaml
g++ -std=c++17 -Wall -Wextra -Werror -I components tests/bq769x0_registers_test.cpp -o /tmp/bq769x0_test && /tmp/bq769x0_test
.venv/bin/clang-format --dry-run --Werror components/bq769x0/*.h components/bq769x0/*.cpp tests/*.cpp
.venv/bin/ruff check components && .venv/bin/ruff format --check components
.venv/bin/yamllint --strict .
```

## Demo board

The demo board this is tested on consists of a ESP32-C3 connected to the afe with:

- IO10 -> Alert (active high at the GPIO)
- IO4 -> Wake (connected to TS1 to allow pulsing to wake)
- IO5 -> SCL
- IO6 -> SDA
