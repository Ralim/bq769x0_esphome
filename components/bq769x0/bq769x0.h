#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#include <cmath>

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include "bq769x0_registers.h"

namespace esphome::bq769x0 {

static constexpr uint8_t MAX_CELLS = 15;

class BQ769X0Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_model(Model model) { this->model_ = model; }
  void set_cell_count(uint8_t cell_count) { this->cell_count_ = cell_count; }
  void set_crc_enabled(bool crc_enabled) { this->crc_enabled_ = crc_enabled; }
  void set_shunt_resistance(float shunt_resistance) { this->shunt_resistance_ = shunt_resistance; }
  void set_alert_pin(GPIOPin *pin) { this->alert_pin_ = pin; }
  void set_wakeup_pin(GPIOPin *pin) { this->wakeup_pin_ = pin; }
  void set_balancing_enabled(bool enabled) { this->balancing_enabled_ = enabled; }
  void set_balancing_minimum_delta(float volts) { this->balancing_min_delta_mv_ = lroundf(volts * 1000.0f); }
  void set_balancing_minimum_voltage(float volts) { this->balancing_min_voltage_mv_ = lroundf(volts * 1000.0f); }
  void set_minimum_cell_voltage(float volts) { this->min_cell_mv_ = lroundf(volts * 1000.0f); }
  void set_maximum_cell_voltage(float volts) { this->max_cell_mv_ = lroundf(volts * 1000.0f); }
  void set_maximum_charge_current(float amps) { this->max_charge_a_ = amps; }
  void set_maximum_discharge_current(float amps) { this->max_discharge_a_ = amps; }
  void set_overcurrent_recovery_time(uint32_t ms) { this->overcurrent_recovery_ms_ = ms; }

#ifdef USE_SENSOR
  // `cell` is zero-based.
  void set_cell_voltage_sensor(uint8_t cell, sensor::Sensor *sensor) { this->cell_voltage_sensors_[cell] = sensor; }
  void set_pack_voltage_sensor(sensor::Sensor *sensor) { this->pack_voltage_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { this->current_sensor_ = sensor; }
  void set_max_cell_delta_sensor(sensor::Sensor *sensor) { this->max_cell_delta_sensor_ = sensor; }
#endif
#ifdef USE_BINARY_SENSOR
  // `cell` is zero-based.
  void set_balancing_sensor(uint8_t cell, binary_sensor::BinarySensor *sensor) {
    this->balancing_sensors_[cell] = sensor;
  }
  void set_charge_fet_sensor(binary_sensor::BinarySensor *sensor) { this->charge_fet_sensor_ = sensor; }
  void set_discharge_fet_sensor(binary_sensor::BinarySensor *sensor) { this->discharge_fet_sensor_ = sensor; }
#endif
#ifdef USE_TEXT_SENSOR
  void set_alarms_sensor(text_sensor::TextSensor *sensor) { this->alarms_sensor_ = sensor; }
#endif

 protected:
  // Register access that transparently handles the CRC byte on CRC-enabled parts.
  bool read_registers_(uint8_t a_register, uint8_t *data, size_t len);
  bool read_register_(uint8_t a_register, uint8_t *value) { return this->read_registers_(a_register, value, 1); }
  bool write_register_(uint8_t a_register, uint8_t value);

  void wake_device_();
  // Reads, reports and clears SYS_STAT, latching discharge faults and reading the current when it is fresh.
  bool process_status_();
  // Each returns false if the device could not be read or the reading is not trustworthy.
  bool update_cells_();
  bool update_pack_voltage_();
  bool read_current_();
  bool update_current_();
  // Runs one odd-even balancing cycle, turning balancing off if the cell readings are not valid.
  bool run_balancing_();
  bool write_balancing_(uint16_t inputs);
  bool update_balancing_();
  // FET bits (CHG_ON/DSG_ON) that have a voltage or current limit configured.
  uint8_t controlled_fets_() const;
  // Holds FETs off while the current exceeds its limit; returns true if a FET was newly blocked.
  bool check_current_limits_();
  // Switches the controlled CHG/DSG FETs according to the cell voltage and current limits, and publishes the
  // resulting FET state.
  bool update_fets_();
  void publish_fet_state_(uint8_t sys_ctrl2);
  // Active alarm::* bits from the latest readings and protection state.
  uint8_t active_alarms_() const;
  void publish_alarms_();

  Model model_{Model::BQ76920};
  uint8_t cell_count_{0};
  bool crc_enabled_{false};
  float shunt_resistance_{0.0f};
  GPIOPin *alert_pin_{nullptr};
  GPIOPin *wakeup_pin_{nullptr};

  bool balancing_enabled_{false};
  int32_t balancing_min_delta_mv_{0};
  int32_t balancing_min_voltage_mv_{0};
  bool balance_odd_turn_{false};

  // Cell voltage limits for the DSG and CHG FETs, 0 when that FET is not controlled.
  int32_t min_cell_mv_{0};
  int32_t max_cell_mv_{0};
  // Current limits for the CHG and DSG FETs in A, 0 when not limited.
  float max_charge_a_{0.0f};
  float max_discharge_a_{0.0f};
  uint32_t overcurrent_recovery_ms_{0};
  // FET bits held off by an overcurrent, each until `overcurrent_recovery_ms_` after its last trip.
  uint8_t overcurrent_fets_{0};
  uint32_t charge_trip_ms_{0};
  uint32_t discharge_trip_ms_{0};
  // Latest coulomb counter reading in A, positive when charging.
  float current_a_{0.0f};
  bool current_valid_{false};
  // Set by an SCD/OCD fault, cleared once the load is detected as removed.
  bool load_fault_{false};

  uint16_t adc_gain_uv_{0};
  int8_t adc_offset_mv_{0};
  // Latest reading of every cell input, only valid when `cells_valid_`.
  int32_t input_mv_[MAX_CELLS]{};
  // Cell inputs (bit 0 = VC1) found to have a cell connected, only valid when `cells_valid_`.
  uint16_t cell_inputs_{0};
  bool cells_valid_{false};

#ifdef USE_SENSOR
  sensor::Sensor *cell_voltage_sensors_[MAX_CELLS]{};
  sensor::Sensor *pack_voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *max_cell_delta_sensor_{nullptr};
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *balancing_sensors_[MAX_CELLS]{};
  binary_sensor::BinarySensor *charge_fet_sensor_{nullptr};
  binary_sensor::BinarySensor *discharge_fet_sensor_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *alarms_sensor_{nullptr};
  // Last published alarm bits, 0xFF before the first publish.
  uint8_t published_alarms_{0xFF};
#endif
};

}  // namespace esphome::bq769x0
