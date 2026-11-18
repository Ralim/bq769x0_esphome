# BQ769x0 driver for ESPHome

External ESPHome component for the TI bq76920 / bq76930 / bq76940 battery monitor AFEs (I2C).

## Usage

```yaml
external_components:
  - source: github://ralim/bq769x0_esphome
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
    cell_1: # cell_1 .. cell_<cell_count>
      name: Cell 1 Voltage

binary_sensor:
  - platform: bq769x0
    bq769x0_id: bms
    balancing_cell_1: # balancing_cell_1 .. balancing_cell_<cell_count>
      name: Cell 1 Balancing
```

Cells are numbered from the bottom of the stack, skipping cell inputs that are shorted (unused), so on a
bq76920 wired for 4S with VC4 shorted, `cell_4` reads VC5.

Balancing uses the classic odd-even scheme: on each update the cells needing balancing are
found, then only the odd cells (2nd, 4th, ...) or the even cells are balanced, alternating each update, so two
adjacent cells are never balanced at once. If one half has nothing to balance the other half runs instead.
Balancing is turned off whenever the cell readings are invalid or the spread is implausibly large.

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
