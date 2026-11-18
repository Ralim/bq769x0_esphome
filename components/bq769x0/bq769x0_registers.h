#pragma once

// Register map and pure helpers for the TI bq769x0 family (bq76920 / bq76930 / bq76940).
// This header intentionally has no ESPHome dependencies so it can be unit tested on the host.

#include <cstddef>
#include <cstdint>
#include <climits>

namespace esphome::bq769x0 {

enum class Model : uint8_t {
  BQ76920,  // 3-5 cells
  BQ76930,  // 6-10 cells
  BQ76940,  // 9-15 cells
};

constexpr uint8_t max_cells(Model model) {
  switch (model) {
    case Model::BQ76920:
      return 5;
    case Model::BQ76930:
      return 10;
    case Model::BQ76940:
    default:
      return 15;
  }
}

// One CELLBALx register per group of 5 cell inputs.
constexpr uint8_t cell_groups(Model model) { return max_cells(model) / 5; }

namespace reg {
static constexpr uint8_t SYS_STAT = 0x00;
static constexpr uint8_t CELLBAL1 = 0x01;
static constexpr uint8_t CELLBAL2 = 0x02;
static constexpr uint8_t CELLBAL3 = 0x03;
static constexpr uint8_t SYS_CTRL1 = 0x04;
static constexpr uint8_t SYS_CTRL2 = 0x05;
static constexpr uint8_t PROTECT1 = 0x06;
static constexpr uint8_t PROTECT2 = 0x07;
static constexpr uint8_t PROTECT3 = 0x08;
static constexpr uint8_t OV_TRIP = 0x09;
static constexpr uint8_t UV_TRIP = 0x0A;
static constexpr uint8_t CC_CFG = 0x0B;
// VC1_HI..VC15_LO are contiguous HI/LO pairs starting here.
static constexpr uint8_t VC1_HI = 0x0C;
static constexpr uint8_t BAT_HI = 0x2A;
static constexpr uint8_t BAT_LO = 0x2B;
static constexpr uint8_t TS1_HI = 0x2C;
static constexpr uint8_t TS1_LO = 0x2D;
static constexpr uint8_t TS2_HI = 0x2E;
static constexpr uint8_t TS2_LO = 0x2F;
static constexpr uint8_t TS3_HI = 0x30;
static constexpr uint8_t TS3_LO = 0x31;
static constexpr uint8_t CC_HI = 0x32;
static constexpr uint8_t CC_LO = 0x33;
static constexpr uint8_t ADCGAIN1 = 0x50;
static constexpr uint8_t ADCOFFSET = 0x51;
static constexpr uint8_t ADCGAIN2 = 0x59;
}  // namespace reg

namespace sys_stat {
static constexpr uint8_t CC_READY = 1 << 7;
static constexpr uint8_t DEVICE_XREADY = 1 << 5;
static constexpr uint8_t OVRD_ALERT = 1 << 4;
static constexpr uint8_t UV = 1 << 3;
static constexpr uint8_t OV = 1 << 2;
static constexpr uint8_t SCD = 1 << 1;
static constexpr uint8_t OCD = 1 << 0;
}  // namespace sys_stat

namespace sys_ctrl1 {
static constexpr uint8_t LOAD_PRESENT = 1 << 7;
static constexpr uint8_t ADC_EN = 1 << 4;
static constexpr uint8_t TEMP_SEL = 1 << 3;
static constexpr uint8_t SHUT_A = 1 << 1;
static constexpr uint8_t SHUT_B = 1 << 0;
}  // namespace sys_ctrl1

namespace sys_ctrl2 {
static constexpr uint8_t DELAY_DIS = 1 << 7;
static constexpr uint8_t CC_EN = 1 << 6;
static constexpr uint8_t CC_ONESHOT = 1 << 5;
static constexpr uint8_t DSG_ON = 1 << 1;
static constexpr uint8_t CHG_ON = 1 << 0;
}  // namespace sys_ctrl2

// CC_CFG must be programmed to this value after every boot.
static constexpr uint8_t CC_CFG_VALUE = 0x19;

// CRC-8, polynomial x^8 + x^2 + x + 1 (0x07), initial value 0, as used by the CRC-enabled part variants.
constexpr uint8_t crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

constexpr uint8_t crc8(const uint8_t *data, size_t len, uint8_t crc = 0) {
  for (size_t i = 0; i < len; i++) {
    crc = crc8_update(crc, data[i]);
  }
  return crc;
}

// CRC for a single register write: covers the 8-bit write address, register and data byte.
constexpr uint8_t write_crc(uint8_t address, uint8_t a_register, uint8_t value) {
  return crc8_update(crc8_update(crc8_update(0, static_cast<uint8_t>(address << 1)), a_register), value);
}

// CRC for the byte at `index` of a read: the first byte also covers the 8-bit read address,
// every following byte is covered on its own.
constexpr uint8_t read_crc(uint8_t address, size_t index, uint8_t value) {
  return index == 0 ? crc8_update(crc8_update(0, static_cast<uint8_t>((address << 1) | 1)), value)
                    : crc8_update(0, value);
}

// ADC gain in uV/LSB: 365 + ADCGAIN<4:0>, where ADCGAIN<4:3> = ADCGAIN1<3:2> and ADCGAIN<2:0> = ADCGAIN2<7:5>.
constexpr uint16_t decode_adc_gain_uv(uint8_t adcgain1, uint8_t adcgain2) {
  return static_cast<uint16_t>(365 + (((adcgain1 & 0x0C) << 1) | ((adcgain2 & 0xE0) >> 5)));
}

// ADC offset in mV, stored as a signed 8-bit value.
constexpr int8_t decode_adc_offset_mv(uint8_t adcoffset) { return static_cast<int8_t>(adcoffset); }

// Combine a HI/LO register pair into the 14-bit ADC reading.
constexpr uint16_t adc_raw14(uint8_t hi, uint8_t lo) { return static_cast<uint16_t>(((hi & 0x3F) << 8) | lo); }

// Cell voltage in mV from a 14-bit ADC reading and the factory gain/offset.
constexpr int32_t cell_voltage_mv(uint16_t raw, uint16_t gain_uv, int8_t offset_mv) {
  return (static_cast<int32_t>(raw) * gain_uv) / 1000 + offset_mv;
}

// Hardware protection. OCD/SCD thresholds are in mV across the sense resistor, indexed by the register code, for
// PROTECT1[RSNS] = 0 and 1. The RSNS bit selects the range for both at once.
static constexpr uint8_t OCD_THRESHOLDS_MV[2][16] = {
    {8, 11, 14, 17, 19, 22, 25, 28, 31, 33, 36, 39, 42, 44, 47, 50},
    {17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100},
};
static constexpr uint8_t SCD_THRESHOLDS_MV[2][8] = {
    {22, 33, 44, 56, 67, 78, 89, 100},
    {44, 67, 89, 111, 133, 155, 178, 200},
};
// Delays per register code.
static constexpr uint16_t SCD_DELAYS_US[4] = {70, 100, 200, 400};
static constexpr uint16_t OCD_DELAYS_MS[8] = {8, 20, 40, 80, 160, 320, 640, 1280};
static constexpr uint8_t OV_DELAYS_S[4] = {1, 2, 4, 8};
static constexpr uint8_t UV_DELAYS_S[4] = {1, 4, 8, 16};

// Whether the upper OCD/SCD range (RSNS = 1) is needed to reach the requested thresholds.
constexpr bool protection_upper_range(float ocd_mv, float scd_mv) {
  return ocd_mv > OCD_THRESHOLDS_MV[0][15] || scd_mv > SCD_THRESHOLDS_MV[0][7];
}

// Code of the smallest threshold at or above `mv`, so the hardware never trips below the requested value. Values
// above the table use the largest threshold.
constexpr uint8_t threshold_code_at_or_above(const uint8_t *thresholds_mv, uint8_t count, float mv) {
  for (uint8_t i = 0; i < count; i++) {
    if (thresholds_mv[i] + 0.001f >= mv) {
      return i;
    }
  }
  return static_cast<uint8_t>(count - 1);
}

constexpr uint8_t protect1_value(bool upper_range, uint8_t scd_delay_code, uint8_t scd_code) {
  return static_cast<uint8_t>((upper_range ? 0x80 : 0) | ((scd_delay_code & 0x03) << 3) | (scd_code & 0x07));
}

constexpr uint8_t protect2_value(uint8_t ocd_delay_code, uint8_t ocd_code) {
  return static_cast<uint8_t>(((ocd_delay_code & 0x07) << 4) | (ocd_code & 0x0F));
}

// PROTECT3 bits 3:0 are reserved and must stay 0.
constexpr uint8_t protect3_value(uint8_t uv_delay_code, uint8_t ov_delay_code) {
  return static_cast<uint8_t>(((uv_delay_code & 0x03) << 6) | ((ov_delay_code & 0x03) << 4));
}

// Device defaults for the trip registers, kept when no limit is configured.
static constexpr uint8_t OV_TRIP_DEFAULT = 0xAC;
static constexpr uint8_t UV_TRIP_DEFAULT = 0x97;

// OV/UV trip registers hold the middle 8 bits of a 14-bit ADC value: OV is 10-OV_TRIP-1000 and UV is
// 01-UV_TRIP-0000.
constexpr uint16_t ov_trip_adc(uint8_t code) { return static_cast<uint16_t>(0x2000 | (code << 4) | 0x8); }
constexpr uint16_t uv_trip_adc(uint8_t code) { return static_cast<uint16_t>(0x1000 | (code << 4)); }

constexpr int32_t ov_trip_mv(uint8_t code, uint16_t gain_uv, int8_t offset_mv) {
  return cell_voltage_mv(ov_trip_adc(code), gain_uv, offset_mv);
}
constexpr int32_t uv_trip_mv(uint8_t code, uint16_t gain_uv, int8_t offset_mv) {
  return cell_voltage_mv(uv_trip_adc(code), gain_uv, offset_mv);
}

// Smallest OV_TRIP whose threshold is at or above `mv`, clamped to the register range.
constexpr uint8_t ov_trip_code(int32_t mv, uint16_t gain_uv, int8_t offset_mv) {
  for (int code = 0; code < 256; code++) {
    if (ov_trip_mv(static_cast<uint8_t>(code), gain_uv, offset_mv) >= mv) {
      return static_cast<uint8_t>(code);
    }
  }
  return 0xFF;
}

// Largest UV_TRIP whose threshold is at or below `mv`, clamped to the register range.
constexpr uint8_t uv_trip_code(int32_t mv, uint16_t gain_uv, int8_t offset_mv) {
  for (int code = 255; code >= 0; code--) {
    if (uv_trip_mv(static_cast<uint8_t>(code), gain_uv, offset_mv) <= mv) {
      return static_cast<uint8_t>(code);
    }
  }
  return 0x00;
}

// Pack voltage in mV from the 16-bit BAT reading: 4 x GAIN x ADC + (#cells x OFFSET).
constexpr int32_t pack_voltage_mv(uint16_t raw, uint16_t gain_uv, int8_t offset_mv, uint8_t cells) {
  return (4 * static_cast<int32_t>(raw) * gain_uv) / 1000 + cells * offset_mv;
}

// Coulomb counter LSB in uV across the sense resistor.
static constexpr float CC_LSB_UV = 8.44f;

// Pack current in A from the signed coulomb counter reading. Positive is charging.
constexpr float cc_current_a(int16_t raw, float shunt_ohm) { return raw * CC_LSB_UV * 1e-6f / shunt_ohm; }

// Inputs below this read as shorted, which is how unused cell positions are wired.
static constexpr int32_t CELL_PRESENT_THRESHOLD_MV = 500;

// Bitmask of cell inputs (bit 0 = VC1) whose voltage indicates a connected cell.
constexpr uint16_t populated_inputs(const int32_t *input_mv, uint8_t inputs) {
  uint16_t mask = 0;
  for (uint8_t i = 0; i < inputs; i++) {
    if (input_mv[i] >= CELL_PRESENT_THRESHOLD_MV) {
      mask |= static_cast<uint16_t>(1u << i);
    }
  }
  return mask;
}

constexpr uint8_t count_bits(uint16_t mask) {
  uint8_t count = 0;
  for (; mask != 0; mask &= static_cast<uint16_t>(mask - 1)) {
    count++;
  }
  return count;
}

// Input index (0 = VC1) of the zero-based `cell`, i.e. the position of the cell-th set bit, or -1.
constexpr int8_t input_for_cell(uint16_t mask, uint8_t cell) {
  for (int8_t i = 0; i < 16; i++) {
    if (mask & (1u << i)) {
      if (cell == 0) {
        return i;
      }
      cell--;
    }
  }
  return -1;
}

// Inputs (bit 0 = VC1) of the populated cells with an odd zero-based cell index, i.e. the 2nd, 4th, ... cell.
// Odd-even balancing alternates between these and the remaining cells. Alternating by cell rather than by
// input keeps physically adjacent cells apart when unused inputs are shorted (e.g. 4S on a bq76920 puts cell 3
// on VC3 and cell 4 on VC5), and since cells in the same phase are always at least two inputs apart, no two
// adjacent inputs are balanced together either, as the datasheet requires.
constexpr uint16_t odd_cell_inputs(uint16_t populated) {
  uint16_t mask = 0;
  bool odd = false;
  for (uint8_t i = 0; i < 16; i++) {
    if (populated & (1u << i)) {
      if (odd) {
        mask |= static_cast<uint16_t>(1u << i);
      }
      odd = !odd;
    }
  }
  return mask;
}

// Lowest and highest voltage among the populated inputs. Both are 0 when nothing is populated.
constexpr int32_t lowest_cell_mv(const int32_t *input_mv, uint16_t populated, uint8_t inputs) {
  int32_t lowest = INT32_MAX;
  for (uint8_t i = 0; i < inputs; i++) {
    if ((populated & (1u << i)) && input_mv[i] < lowest) {
      lowest = input_mv[i];
    }
  }
  return lowest == INT32_MAX ? 0 : lowest;
}

constexpr int32_t highest_cell_mv(const int32_t *input_mv, uint16_t populated, uint8_t inputs) {
  int32_t highest = INT32_MIN;
  for (uint8_t i = 0; i < inputs; i++) {
    if ((populated & (1u << i)) && input_mv[i] > highest) {
      highest = input_mv[i];
    }
  }
  return highest == INT32_MIN ? 0 : highest;
}

// Maximum difference between any two populated cells.
constexpr int32_t max_cell_delta_mv(const int32_t *input_mv, uint16_t populated, uint8_t inputs) {
  return highest_cell_mv(input_mv, populated, inputs) - lowest_cell_mv(input_mv, populated, inputs);
}

// Odd-even balancing: the cells (as a bitmask of inputs, bit 0 = VC1) that need bleeding down. A cell needs
// balancing when it is above both `min_voltage_mv` and the lowest cell by more than `min_delta_mv`.
// Nothing is balanced when the spread is implausibly large (at least the lowest cell voltage), which
// indicates a bad reading rather than an imbalance.
constexpr uint16_t cells_needing_balance(const int32_t *input_mv, uint16_t populated, uint8_t inputs,
                                         int32_t min_delta_mv, int32_t min_voltage_mv) {
  const int32_t lowest = lowest_cell_mv(input_mv, populated, inputs);
  const int32_t highest = highest_cell_mv(input_mv, populated, inputs);
  if (populated == 0 || highest <= min_voltage_mv) {
    return 0;
  }
  const int32_t delta = highest - lowest;
  if (delta <= min_delta_mv || delta >= lowest) {
    return 0;
  }
  uint16_t mask = 0;
  for (uint8_t i = 0; i < inputs; i++) {
    if ((populated & (1u << i)) && input_mv[i] > lowest + min_delta_mv && input_mv[i] > min_voltage_mv) {
      mask |= static_cast<uint16_t>(1u << i);
    }
  }
  return mask;
}

// The subset of `needing` to balance this cycle: the odd cells on odd turns and the even cells otherwise.
// When the selected half has nothing to balance the other half runs instead, so no cycle is wasted.
constexpr uint16_t balance_phase(uint16_t needing, uint16_t odd_inputs, bool odd_turn) {
  const uint16_t odd = needing & odd_inputs;
  const uint16_t even = needing & static_cast<uint16_t>(~odd_inputs);
  const uint16_t preferred = odd_turn ? odd : even;
  return preferred != 0 ? preferred : (odd_turn ? even : odd);
}

// Value for CELLBAL<group + 1> from an input mask with bit 0 = VC1.
constexpr uint8_t cellbal_register_value(uint16_t mask, uint8_t group) {
  return static_cast<uint8_t>((mask >> (group * 5)) & 0x1F);
}

// Combine CELLBAL1..CELLBALn (5 inputs each) into one mask with bit 0 = VC1.
constexpr uint16_t balancing_inputs(const uint8_t *cellbal, uint8_t registers) {
  uint16_t mask = 0;
  for (uint8_t i = 0; i < registers; i++) {
    mask |= static_cast<uint16_t>((cellbal[i] & 0x1F) << (i * 5));
  }
  return mask;
}

// FET bits (CHG_ON/DSG_ON) the cell voltage limits allow to be on: DSG while the lowest cell is at or above
// `min_cell_mv` and CHG while the highest cell is at or below `max_cell_mv`. A limit of 0 never blocks.
constexpr uint8_t fets_allowed_by_voltage(int32_t lowest_mv, int32_t highest_mv, int32_t min_cell_mv,
                                          int32_t max_cell_mv) {
  uint8_t allowed = 0;
  if (min_cell_mv <= 0 || lowest_mv >= min_cell_mv) {
    allowed |= sys_ctrl2::DSG_ON;
  }
  if (max_cell_mv <= 0 || highest_mv <= max_cell_mv) {
    allowed |= sys_ctrl2::CHG_ON;
  }
  return allowed;
}

// FET bits to turn off because the current exceeds a limit. Positive current is charging. A limit of 0 never
// blocks.
constexpr uint8_t fets_blocked_by_current(float current_a, float max_charge_a, float max_discharge_a) {
  uint8_t blocked = 0;
  if (max_charge_a > 0.0f && current_a > max_charge_a) {
    blocked |= sys_ctrl2::CHG_ON;
  }
  if (max_discharge_a > 0.0f && -current_a > max_discharge_a) {
    blocked |= sys_ctrl2::DSG_ON;
  }
  return blocked;
}

// SYS_CTRL2 with the `controlled` FET bits replaced by those in `on`; all other bits are preserved.
constexpr uint8_t apply_fets(uint8_t sys_ctrl2, uint8_t controlled, uint8_t on) {
  return static_cast<uint8_t>((sys_ctrl2 & ~controlled) | (on & controlled));
}

namespace alarm {
static constexpr uint8_t OVER_CELL_VOLTAGE = 1 << 0;
static constexpr uint8_t UNDER_CELL_VOLTAGE = 1 << 1;
static constexpr uint8_t OVER_CHARGE_CURRENT = 1 << 2;
static constexpr uint8_t OVER_DISCHARGE_CURRENT = 1 << 3;
// Device short-circuit protection tripped, held until the load is removed.
static constexpr uint8_t SHORT_CIRCUIT = 1 << 4;
static constexpr uint8_t COUNT = 5;
}  // namespace alarm

constexpr const char *alarm_name(uint8_t index) {
  switch (index) {
    case 0:
      return "Over Cell Voltage";
    case 1:
      return "Under Cell Voltage";
    case 2:
      return "Over Charge Current";
    case 3:
      return "Over Discharge Current";
    case 4:
      return "Short Circuit";
    default:
      return "Unknown";
  }
}

constexpr size_t alarm_text_length(const char *text) {
  size_t len = 0;
  while (text[len] != '\0') {
    len++;
  }
  return len;
}

// Buffer size that fits every alarm at once, including separators and the terminator.
constexpr size_t alarms_text_size() {
  size_t size = 1;
  for (uint8_t i = 0; i < alarm::COUNT; i++) {
    size += alarm_text_length(alarm_name(i)) + 2;
  }
  return size;
}

// Voltage alarms for the given cell extremes. A limit of 0 never alarms.
constexpr uint8_t voltage_alarms(int32_t lowest_mv, int32_t highest_mv, int32_t min_cell_mv, int32_t max_cell_mv) {
  uint8_t alarms = 0;
  if (max_cell_mv > 0 && highest_mv > max_cell_mv) {
    alarms |= alarm::OVER_CELL_VOLTAGE;
  }
  if (min_cell_mv > 0 && lowest_mv < min_cell_mv) {
    alarms |= alarm::UNDER_CELL_VOLTAGE;
  }
  return alarms;
}

// Text for an alarm bitmask: the active alarm names separated by ", ", or "OK" when there are none.
// Returns the length written, truncating to fit `size`.
inline size_t format_alarms(uint8_t alarms, char *out, size_t size) {
  if (size == 0) {
    return 0;
  }
  size_t pos = 0;
  auto append = [&](const char *text) {
    for (; *text != '\0' && pos + 1 < size; text++) {
      out[pos++] = *text;
    }
  };
  if (alarms == 0) {
    append("OK");
  }
  for (uint8_t i = 0; i < alarm::COUNT; i++) {
    if (alarms & (1u << i)) {
      if (pos != 0) {
        append(", ");
      }
      append(alarm_name(i));
    }
  }
  out[pos] = '\0';
  return pos;
}

}  // namespace esphome::bq769x0
