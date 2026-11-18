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

#ifdef USE_SENSOR
  // `cell` is zero-based.
  void set_cell_voltage_sensor(uint8_t cell, sensor::Sensor *sensor) { this->cell_voltage_sensors_[cell] = sensor; }
  void set_pack_voltage_sensor(sensor::Sensor *sensor) { this->pack_voltage_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { this->current_sensor_ = sensor; }
#endif
#ifdef USE_BINARY_SENSOR
  // `cell` is zero-based.
  void set_balancing_sensor(uint8_t cell, binary_sensor::BinarySensor *sensor) {
    this->balancing_sensors_[cell] = sensor;
  }
#endif

 protected:
  // Register access that transparently handles the CRC byte on CRC-enabled parts.
  bool read_registers_(uint8_t a_register, uint8_t *data, size_t len);
  bool read_register_(uint8_t a_register, uint8_t *value) { return this->read_registers_(a_register, value, 1); }
  bool write_register_(uint8_t a_register, uint8_t value);

  void wake_device_();
  void handle_alert_();
  // Each returns false if the device could not be read or the reading is not trustworthy.
  bool update_cells_();
  bool update_pack_voltage_();
  bool update_current_();
  // Runs one odd-even balancing cycle, turning balancing off if the cell readings are not valid.
  bool run_balancing_();
  bool write_balancing_(uint16_t inputs);
  bool update_balancing_();

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
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *balancing_sensors_[MAX_CELLS]{};
#endif
};

}  // namespace esphome::bq769x0
