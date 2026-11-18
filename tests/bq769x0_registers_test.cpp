// Host-side unit tests for the dependency-free bq769x0 helpers.
// Built and run by the unit-tests job in .github/workflows/ci.yml.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include "bq769x0/bq769x0_registers.h"

using namespace esphome::bq769x0;

static int failures = 0;

#define EXPECT_EQ(actual, expected) \
  do { \
    const long long a_ = static_cast<long long>(actual); \
    const long long e_ = static_cast<long long>(expected); \
    if (a_ != e_) { \
      std::printf("%s:%d: %s == %lld, expected %lld\n", __FILE__, __LINE__, #actual, a_, e_); \
      failures++; \
    } \
  } while (0)

#define EXPECT_STREQ(actual, expected) \
  do { \
    if (std::strcmp((actual), (expected)) != 0) { \
      std::printf("%s:%d: %s == \"%s\", expected \"%s\"\n", __FILE__, __LINE__, #actual, (actual), (expected)); \
      failures++; \
    } \
  } while (0)

static void test_crc8() {
  // Standard CRC-8 (poly 0x07, init 0) check value.
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(crc8(check, sizeof(check)), 0xF4);
  EXPECT_EQ(crc8_update(0, 0x00), 0x00);
  EXPECT_EQ(crc8_update(0, 0x01), 0x07);
  // Chaining must match a single pass over the same bytes.
  EXPECT_EQ(crc8(check + 4, 5, crc8(check, 4)), 0xF4);
}

static void test_write_crc() {
  const uint8_t address = 0x08;
  const uint8_t frame[] = {address << 1, reg::CC_CFG, CC_CFG_VALUE};
  EXPECT_EQ(write_crc(address, reg::CC_CFG, CC_CFG_VALUE), crc8(frame, sizeof(frame)));
}

static void test_read_crc() {
  const uint8_t address = 0x18;
  const uint8_t first[] = {(address << 1) | 1, 0x5A};
  EXPECT_EQ(read_crc(address, 0, 0x5A), crc8(first, sizeof(first)));
  EXPECT_EQ(read_crc(address, 1, 0x5A), crc8_update(0, 0x5A));
  EXPECT_EQ(read_crc(address, 7, 0x5A), crc8_update(0, 0x5A));
}

static void test_adc_gain() {
  EXPECT_EQ(decode_adc_gain_uv(0x00, 0x00), 365);
  EXPECT_EQ(decode_adc_gain_uv(0x0C, 0xE0), 396);
  EXPECT_EQ(decode_adc_gain_uv(0x04, 0x20), 374);
  // Bits outside the gain fields must be ignored.
  EXPECT_EQ(decode_adc_gain_uv(0xF3, 0x1F), 365);
}

static void test_adc_offset() {
  EXPECT_EQ(decode_adc_offset_mv(0x00), 0);
  EXPECT_EQ(decode_adc_offset_mv(0x7F), 127);
  EXPECT_EQ(decode_adc_offset_mv(0xFF), -1);
  EXPECT_EQ(decode_adc_offset_mv(0x80), -128);
}

static void test_cell_voltage() {
  EXPECT_EQ(adc_raw14(0xFF, 0xFF), 0x3FFF);
  EXPECT_EQ(adc_raw14(0x12, 0x34), 0x1234);
  // 0x1F40 = 8000 counts at 380 uV/LSB = 3040 mV, plus offset.
  EXPECT_EQ(cell_voltage_mv(0x1F40, 380, 0), 3040);
  EXPECT_EQ(cell_voltage_mv(0x1F40, 380, -12), 3028);
  EXPECT_EQ(cell_voltage_mv(0x3FFF, 396, 127), 6614);
}

static void test_max_cells() {
  EXPECT_EQ(max_cells(Model::BQ76920), 5);
  EXPECT_EQ(max_cells(Model::BQ76930), 10);
  EXPECT_EQ(max_cells(Model::BQ76940), 15);
  EXPECT_EQ(cell_groups(Model::BQ76920), 1);
  EXPECT_EQ(cell_groups(Model::BQ76930), 2);
  EXPECT_EQ(cell_groups(Model::BQ76940), 3);
}

static void test_pack_voltage() {
  // 0x3500 = 13568 counts * 4 * 377 uV = 20460 mV, plus 4 cells * 47 mV.
  EXPECT_EQ(pack_voltage_mv(0x3500, 377, 47, 4), 20648);
  EXPECT_EQ(pack_voltage_mv(0x3500, 377, -10, 4), 20420);
}

static void test_current() {
  EXPECT_EQ(cc_current_a(0, 0.001f), 0);
  // 1000 counts * 8.44 uV / 1 mOhm = 8.44 A; compare in mA to avoid float equality.
  EXPECT_EQ(cc_current_a(1000, 0.001f) * 1000 + 0.5f, 8440);
  EXPECT_EQ(cc_current_a(-1000, 0.002f) * 1000 - 0.5f, -4220);
}

static void test_cell_detection() {
  // bq76920 wired for 4 cells: VC4 is shorted to VC3.
  const int32_t four_s[] = {3346, 3360, 3352, 12, 3340};
  const uint16_t mask = populated_inputs(four_s, 5);
  EXPECT_EQ(mask, 0b10111);
  EXPECT_EQ(count_bits(mask), 4);
  EXPECT_EQ(input_for_cell(mask, 0), 0);
  EXPECT_EQ(input_for_cell(mask, 2), 2);
  EXPECT_EQ(input_for_cell(mask, 3), 4);
  EXPECT_EQ(input_for_cell(mask, 4), -1);

  // Anything below the threshold, including a negative reading from the offset, is unpopulated.
  const int32_t edge[] = {CELL_PRESENT_THRESHOLD_MV, CELL_PRESENT_THRESHOLD_MV - 1, -20};
  EXPECT_EQ(populated_inputs(edge, 3), 0b001);
  EXPECT_EQ(count_bits(0), 0);
  EXPECT_EQ(count_bits(0x7FFF), 15);
}

static void test_balancing_inputs() {
  const uint8_t one[] = {0x05};
  EXPECT_EQ(balancing_inputs(one, 1), 0b00101);
  // Reserved upper bits are ignored and each register covers the next 5 inputs.
  const uint8_t three[] = {0xE1, 0x02, 0x10};
  EXPECT_EQ(balancing_inputs(three, 3), (1 << 0) | (1 << 6) | (1 << 14));
}

static void test_odd_cell_inputs() {
  // 5S: alternate inputs.
  EXPECT_EQ(odd_cell_inputs(0b11111), 0b01010);
  // 4S on a bq76920 with VC4 shorted: cells are VC1, VC2, VC3, VC5, so cells 2 and 4 are VC2 and VC5.
  EXPECT_EQ(odd_cell_inputs(0b10111), 0b10010);
  EXPECT_EQ(odd_cell_inputs(0), 0);
}

static void test_cell_extremes() {
  // Unpopulated VC4 (40 mV) is ignored.
  const int32_t four_s[] = {3500, 3530, 3510, 40, 3520};
  EXPECT_EQ(lowest_cell_mv(four_s, 0b10111, 5), 3500);
  EXPECT_EQ(highest_cell_mv(four_s, 0b10111, 5), 3530);
  EXPECT_EQ(max_cell_delta_mv(four_s, 0b10111, 5), 30);
  // Including it would make it the lowest.
  EXPECT_EQ(max_cell_delta_mv(four_s, 0b11111, 5), 3490);
  EXPECT_EQ(max_cell_delta_mv(four_s, 0b00001, 5), 0);
  EXPECT_EQ(lowest_cell_mv(four_s, 0, 5), 0);
  EXPECT_EQ(highest_cell_mv(four_s, 0, 5), 0);
  EXPECT_EQ(max_cell_delta_mv(four_s, 0, 5), 0);
}

static void test_cells_needing_balance() {
  const uint16_t all = 0b11111;
  // Spread 30 mV, cells above 3450 and above lowest + 10 mV need balancing.
  const int32_t five_s[] = {3460, 3480, 3455, 3490, 3470};
  EXPECT_EQ(cells_needing_balance(five_s, all, 5, 10, 3450), 0b11010);
  // Raising the voltage floor excludes the 3470 and 3480 mV cells.
  EXPECT_EQ(cells_needing_balance(five_s, all, 5, 10, 3485), 0b01000);
  // Spread not above the minimum delta: nothing to do.
  EXPECT_EQ(cells_needing_balance(five_s, all, 5, 35, 3450), 0);
  // Highest cell at or below the voltage floor: nothing to do.
  EXPECT_EQ(cells_needing_balance(five_s, all, 5, 10, 3490), 0);

  // Unpopulated inputs are ignored, including for the lowest cell. 3510 mV is not above 3500 + 10 mV.
  const int32_t four_s[] = {3500, 3530, 3510, 40, 3520};
  EXPECT_EQ(cells_needing_balance(four_s, 0b10111, 5, 10, 3450), 0b10010);

  // A spread at least as large as the lowest cell indicates a bad reading.
  const int32_t bogus[] = {1500, 3500, 3500, 3500, 3500};
  EXPECT_EQ(cells_needing_balance(bogus, all, 5, 10, 3450), 0);
  EXPECT_EQ(cells_needing_balance(five_s, 0, 5, 10, 3450), 0);
}

static void test_balance_phase() {
  const uint16_t odd = odd_cell_inputs(0b11111);
  EXPECT_EQ(balance_phase(0b11010, odd, false), 0b10000);
  EXPECT_EQ(balance_phase(0b11010, odd, true), 0b01010);
  // An empty half falls back to the other half.
  EXPECT_EQ(balance_phase(0b01010, odd, false), 0b01010);
  EXPECT_EQ(balance_phase(0b00101, odd, true), 0b00101);
  EXPECT_EQ(balance_phase(0, odd, true), 0);

  // No phase ever balances two adjacent cells or two adjacent inputs.
  for (uint16_t populated : {0b11111, 0b10111, 0b10011}) {
    const uint16_t odd_inputs = odd_cell_inputs(populated);
    for (uint16_t needing = 0; needing < 32; needing++) {
      for (bool turn : {false, true}) {
        const uint16_t selected = balance_phase(needing & populated, odd_inputs, turn);
        EXPECT_EQ(selected & (selected >> 1), 0);
        uint8_t cell = 0;
        int8_t previous = -2;
        for (int8_t input = input_for_cell(populated, cell); input >= 0; input = input_for_cell(populated, ++cell)) {
          if (selected & (1u << input)) {
            EXPECT_EQ(previous == cell - 1, false);
            previous = static_cast<int8_t>(cell);
          }
        }
      }
    }
  }
}

static void test_cellbal_register_value() {
  const uint16_t mask = (1 << 0) | (1 << 6) | (1 << 14);
  EXPECT_EQ(cellbal_register_value(mask, 0), 0x01);
  EXPECT_EQ(cellbal_register_value(mask, 1), 0x02);
  EXPECT_EQ(cellbal_register_value(mask, 2), 0x10);
  const uint8_t regs[] = {cellbal_register_value(mask, 0), cellbal_register_value(mask, 1),
                          cellbal_register_value(mask, 2)};
  EXPECT_EQ(balancing_inputs(regs, 3), mask);
}

static void test_fet_limits() {
  constexpr uint8_t CHG = sys_ctrl2::CHG_ON;
  constexpr uint8_t DSG = sys_ctrl2::DSG_ON;
  constexpr uint8_t BOTH = CHG | DSG;
  constexpr uint8_t CC = sys_ctrl2::CC_EN;

  // Voltage limits are inclusive; only exceeding them turns a FET off.
  EXPECT_EQ(fets_allowed_by_voltage(3300, 3400, 2650, 3550), BOTH);
  EXPECT_EQ(fets_allowed_by_voltage(2650, 3550, 2650, 3550), BOTH);
  EXPECT_EQ(fets_allowed_by_voltage(2649, 3400, 2650, 3550), CHG);
  EXPECT_EQ(fets_allowed_by_voltage(3300, 3551, 2650, 3550), DSG);
  EXPECT_EQ(fets_allowed_by_voltage(2000, 3600, 2650, 3550), 0);
  // A limit of 0 never blocks.
  EXPECT_EQ(fets_allowed_by_voltage(2000, 3600, 0, 0), BOTH);

  // Current limits: positive is charging, exactly at the limit is allowed.
  EXPECT_EQ(fets_blocked_by_current(0.0f, 10.0f, 20.0f), 0);
  EXPECT_EQ(fets_blocked_by_current(10.0f, 10.0f, 20.0f), 0);
  EXPECT_EQ(fets_blocked_by_current(10.5f, 10.0f, 20.0f), CHG);
  EXPECT_EQ(fets_blocked_by_current(-20.0f, 10.0f, 20.0f), 0);
  EXPECT_EQ(fets_blocked_by_current(-20.5f, 10.0f, 20.0f), DSG);
  // A charge above the discharge limit only blocks charging, and vice versa.
  EXPECT_EQ(fets_blocked_by_current(25.0f, 30.0f, 20.0f), 0);
  EXPECT_EQ(fets_blocked_by_current(-15.0f, 10.0f, 20.0f), 0);
  EXPECT_EQ(fets_blocked_by_current(1000.0f, 0.0f, 0.0f), 0);
  EXPECT_EQ(fets_blocked_by_current(-1000.0f, 0.0f, 0.0f), 0);

  // Only controlled FET bits change; everything else is preserved.
  EXPECT_EQ(apply_fets(CC, BOTH, BOTH), CC | BOTH);
  EXPECT_EQ(apply_fets(CC | BOTH, BOTH, 0), CC);
  EXPECT_EQ(apply_fets(CC | CHG, DSG, DSG), CC | BOTH);
  EXPECT_EQ(apply_fets(CHG, DSG, 0), CHG);
  EXPECT_EQ(apply_fets(BOTH, 0, 0), BOTH);
}

static void test_alarms() {
  EXPECT_EQ(voltage_alarms(3300, 3400, 2650, 3550), 0);
  // Limits are inclusive, matching the FET control.
  EXPECT_EQ(voltage_alarms(2650, 3550, 2650, 3550), 0);
  EXPECT_EQ(voltage_alarms(3300, 3551, 2650, 3550), alarm::OVER_CELL_VOLTAGE);
  EXPECT_EQ(voltage_alarms(2649, 3400, 2650, 3550), alarm::UNDER_CELL_VOLTAGE);
  EXPECT_EQ(voltage_alarms(2000, 3600, 2650, 3550), alarm::OVER_CELL_VOLTAGE | alarm::UNDER_CELL_VOLTAGE);
  EXPECT_EQ(voltage_alarms(2000, 3600, 0, 0), 0);

  char text[alarms_text_size()];
  EXPECT_EQ(format_alarms(0, text, sizeof(text)), 2);
  EXPECT_STREQ(text, "OK");
  format_alarms(alarm::UNDER_CELL_VOLTAGE, text, sizeof(text));
  EXPECT_STREQ(text, "Under Cell Voltage");
  format_alarms(alarm::OVER_CELL_VOLTAGE | alarm::OVER_DISCHARGE_CURRENT, text, sizeof(text));
  EXPECT_STREQ(text, "Over Cell Voltage, Over Discharge Current");
  // Every alarm at once fits the buffer size the component uses.
  const size_t len = format_alarms(0x1F, text, sizeof(text));
  EXPECT_STREQ(text,
               "Over Cell Voltage, Under Cell Voltage, Over Charge Current, Over Discharge Current, Discharge Fault");
  EXPECT_EQ(len < sizeof(text) - 1, true);
  // Truncation always leaves a terminated string.
  char small[8];
  EXPECT_EQ(format_alarms(alarm::OVER_CELL_VOLTAGE, small, sizeof(small)), 7);
  EXPECT_STREQ(small, "Over Ce");
}

int main() {
  test_crc8();
  test_write_crc();
  test_read_crc();
  test_adc_gain();
  test_adc_offset();
  test_cell_voltage();
  test_max_cells();
  test_pack_voltage();
  test_current();
  test_cell_detection();
  test_balancing_inputs();
  test_odd_cell_inputs();
  test_cell_extremes();
  test_cells_needing_balance();
  test_balance_phase();
  test_cellbal_register_value();
  test_fet_limits();
  test_alarms();
  if (failures != 0) {
    std::printf("%d failure(s)\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("All tests passed\n");
  return EXIT_SUCCESS;
}
