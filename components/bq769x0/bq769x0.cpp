#include "bq769x0.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome::bq769x0 {

static const char *const TAG = "bq769x0";

// Largest register block read in data bytes (VC1_HI..VC15_LO).
static constexpr size_t MAX_READ_LEN = MAX_CELLS * 2;

// Boot pulse applied to TS1 to bring the device out of SHIP mode, and the time it then needs to boot.
static constexpr uint32_t WAKE_PULSE_MS = 2;
static constexpr uint32_t BOOT_DELAY_MS = 10;

static const char *model_to_string(Model model) {
  switch (model) {
    case Model::BQ76920:
      return "BQ76920";
    case Model::BQ76930:
      return "BQ76930";
    case Model::BQ76940:
      return "BQ76940";
    default:
      return "UNKNOWN";
  }
}

void BQ769X0Component::setup() {
  if (this->alert_pin_ != nullptr) {
    this->alert_pin_->setup();
  }
  if (this->wakeup_pin_ != nullptr) {
    this->wakeup_pin_->setup();
    this->wake_device_();
  }

  // Read before writing anything: with the wrong CRC setting a write would spill its CRC byte into the next register.
  uint8_t cc_cfg;
  if (!this->read_register_(reg::CC_CFG, &cc_cfg)) {
    ESP_LOGE(TAG, "Failed to read CC_CFG, check the address and CRC setting");
    this->mark_failed();
    return;
  }
  if (!this->write_register_(reg::CC_CFG, CC_CFG_VALUE)) {
    ESP_LOGE(TAG, "Failed to write CC_CFG");
    this->mark_failed();
    return;
  }
  if (!this->read_register_(reg::CC_CFG, &cc_cfg) || cc_cfg != CC_CFG_VALUE) {
    ESP_LOGE(TAG, "CC_CFG readback mismatch, check the address and CRC setting");
    this->mark_failed();
    return;
  }

  uint8_t adcgain1, adcoffset, adcgain2;
  if (!this->read_register_(reg::ADCGAIN1, &adcgain1) || !this->read_register_(reg::ADCOFFSET, &adcoffset) ||
      !this->read_register_(reg::ADCGAIN2, &adcgain2)) {
    ESP_LOGE(TAG, "Failed to read ADC calibration");
    this->mark_failed();
    return;
  }
  this->adc_gain_uv_ = decode_adc_gain_uv(adcgain1, adcgain2);
  this->adc_offset_mv_ = decode_adc_offset_mv(adcoffset);

  uint8_t sys_ctrl1;
  if (!this->read_register_(reg::SYS_CTRL1, &sys_ctrl1) ||
      !this->write_register_(reg::SYS_CTRL1, sys_ctrl1 | sys_ctrl1::ADC_EN)) {
    ESP_LOGE(TAG, "Failed to enable the ADC");
    this->mark_failed();
    return;
  }

  if (this->shunt_resistance_ > 0.0f) {
    uint8_t sys_ctrl2;
    if (!this->read_register_(reg::SYS_CTRL2, &sys_ctrl2) ||
        !this->write_register_(reg::SYS_CTRL2, sys_ctrl2 | sys_ctrl2::CC_EN)) {
      ESP_LOGE(TAG, "Failed to enable the coulomb counter");
      this->mark_failed();
      return;
    }
  }

  // Start from a known state; balancing is only ever turned on by run_balancing_().
  if (!this->write_balancing_(0)) {
    ESP_LOGE(TAG, "Failed to clear cell balancing");
    this->mark_failed();
    return;
  }
}

void BQ769X0Component::loop() {
  if (this->alert_pin_ != nullptr && this->alert_pin_->digital_read()) {
    this->handle_alert_();
  }
}

void BQ769X0Component::update() {
  bool ok = this->update_cells_();
  ok &= this->update_pack_voltage_();
  ok &= this->update_current_();
  if (this->balancing_enabled_) {
    ok &= this->run_balancing_();
  }
  ok &= this->update_balancing_();
  if (ok) {
    this->status_clear_warning();
  } else {
    this->status_set_warning();
  }
}

bool BQ769X0Component::update_cells_() {
  const uint8_t inputs = max_cells(this->model_);
  uint8_t data[MAX_READ_LEN];
  if (!this->read_registers_(reg::VC1_HI, data, inputs * 2)) {
    ESP_LOGW(TAG, "Failed to read cell voltages");
    this->cells_valid_ = false;
    return false;
  }

  char summary[MAX_CELLS * 6 + 1];
  size_t pos = 0;
  for (uint8_t i = 0; i < inputs; i++) {
    this->input_mv_[i] =
        cell_voltage_mv(adc_raw14(data[i * 2], data[i * 2 + 1]), this->adc_gain_uv_, this->adc_offset_mv_);
    pos += snprintf(summary + pos, sizeof(summary) - pos, " %" PRId32, this->input_mv_[i]);
  }
  ESP_LOGD(TAG, "Cell inputs (mV):%s", summary);

  const uint16_t mask = populated_inputs(this->input_mv_, inputs);
  const uint8_t found = count_bits(mask);
  const bool valid = found == this->cell_count_;
  if (valid != this->cells_valid_ || mask != this->cell_inputs_) {
    if (valid) {
      ESP_LOGI(TAG, "Found %u cells as expected (inputs 0x%04X)", found, mask);
    } else {
      ESP_LOGW(TAG, "Found %u cells (inputs 0x%04X), expected %u", found, mask, this->cell_count_);
    }
  }
  this->cell_inputs_ = mask;
  this->cells_valid_ = valid;

#ifdef USE_SENSOR
  for (uint8_t cell = 0; cell < this->cell_count_; cell++) {
    sensor::Sensor *sensor = this->cell_voltage_sensors_[cell];
    if (sensor == nullptr) {
      continue;
    }
    const int8_t input = input_for_cell(mask, cell);
    sensor->publish_state(valid && input >= 0 ? this->input_mv_[input] / 1000.0f : NAN);
  }
#endif
  return valid;
}

bool BQ769X0Component::update_pack_voltage_() {
#ifdef USE_SENSOR
  if (this->pack_voltage_sensor_ == nullptr) {
    return true;
  }
  uint8_t data[2];
  if (!this->read_registers_(reg::BAT_HI, data, sizeof(data))) {
    ESP_LOGW(TAG, "Failed to read pack voltage");
    this->pack_voltage_sensor_->publish_state(NAN);
    return false;
  }
  const uint16_t raw = static_cast<uint16_t>((data[0] << 8) | data[1]);
  const int32_t mv = pack_voltage_mv(raw, this->adc_gain_uv_, this->adc_offset_mv_, this->cell_count_);
  ESP_LOGD(TAG, "Pack: %" PRId32 " mV", mv);
  this->pack_voltage_sensor_->publish_state(mv / 1000.0f);
#endif
  return true;
}

bool BQ769X0Component::update_current_() {
#ifdef USE_SENSOR
  if (this->current_sensor_ == nullptr) {
    return true;
  }
  uint8_t data[2];
  if (!this->read_registers_(reg::CC_HI, data, sizeof(data))) {
    ESP_LOGW(TAG, "Failed to read pack current");
    this->current_sensor_->publish_state(NAN);
    return false;
  }
  const auto raw = static_cast<int16_t>((data[0] << 8) | data[1]);
  const float current = cc_current_a(raw, this->shunt_resistance_);
  ESP_LOGD(TAG, "Current: %.3f A (CC raw %d)", current, raw);
  this->current_sensor_->publish_state(current);
#endif
  return true;
}

bool BQ769X0Component::run_balancing_() {
  uint16_t selected = 0;
  if (this->cells_valid_) {
    const uint16_t needing = cells_needing_balance(this->input_mv_, this->cell_inputs_, max_cells(this->model_),
                                                   this->balancing_min_delta_mv_, this->balancing_min_voltage_mv_);
    selected = balance_phase(needing, odd_cell_inputs(this->cell_inputs_), this->balance_odd_turn_);
    this->balance_odd_turn_ = !this->balance_odd_turn_;
    if (needing != 0) {
      ESP_LOGD(TAG, "Cells needing balancing: inputs 0x%04X, balancing 0x%04X this cycle", needing, selected);
    }
  }
  if (!this->write_balancing_(selected)) {
    ESP_LOGW(TAG, "Failed to write balancing state");
    return false;
  }
  return true;
}

bool BQ769X0Component::write_balancing_(uint16_t inputs) {
  const uint8_t groups = cell_groups(this->model_);
  for (uint8_t group = 0; group < groups; group++) {
    if (!this->write_register_(reg::CELLBAL1 + group, cellbal_register_value(inputs, group))) {
      return false;
    }
  }
  return true;
}

bool BQ769X0Component::update_balancing_() {
#ifdef USE_BINARY_SENSOR
  uint8_t cellbal[3];
  const uint8_t groups = cell_groups(this->model_);
  if (!this->read_registers_(reg::CELLBAL1, cellbal, groups)) {
    ESP_LOGW(TAG, "Failed to read balancing status");
    return false;
  }
  const uint16_t balancing = balancing_inputs(cellbal, groups);
  ESP_LOGD(TAG, "Balancing inputs: 0x%04X", balancing);
  if (!this->cells_valid_) {
    return true;
  }
  for (uint8_t cell = 0; cell < this->cell_count_; cell++) {
    binary_sensor::BinarySensor *sensor = this->balancing_sensors_[cell];
    const int8_t input = input_for_cell(this->cell_inputs_, cell);
    if (sensor != nullptr && input >= 0) {
      sensor->publish_state((balancing >> input) & 1);
    }
  }
#endif
  return true;
}

void BQ769X0Component::dump_config() {
  ESP_LOGCONFIG(TAG,
                "BQ769X0:\n"
                "  Model: %s\n"
                "  Cell count: %u\n"
                "  CRC: %s\n"
                "  Shunt resistance: %.3f mOhm\n"
                "  ADC gain: %u uV/LSB\n"
                "  ADC offset: %d mV",
                model_to_string(this->model_), this->cell_count_, YESNO(this->crc_enabled_),
                this->shunt_resistance_ * 1000.0f, this->adc_gain_uv_, this->adc_offset_mv_);
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Alert Pin: ", this->alert_pin_);
  LOG_PIN("  Wakeup Pin: ", this->wakeup_pin_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
  LOG_UPDATE_INTERVAL(this);
  if (this->balancing_enabled_) {
    ESP_LOGCONFIG(TAG,
                  "  Balancing: enabled\n"
                  "    Minimum delta: %" PRId32 " mV\n"
                  "    Minimum voltage: %" PRId32 " mV",
                  this->balancing_min_delta_mv_, this->balancing_min_voltage_mv_);
  } else {
    ESP_LOGCONFIG(TAG, "  Balancing: disabled");
  }
#ifdef USE_SENSOR
  for (auto *sensor : this->cell_voltage_sensors_) {
    LOG_SENSOR("  ", "Cell Voltage", sensor);
  }
  LOG_SENSOR("  ", "Pack Voltage", this->pack_voltage_sensor_);
  LOG_SENSOR("  ", "Current", this->current_sensor_);
#endif
#ifdef USE_BINARY_SENSOR
  for (auto *sensor : this->balancing_sensors_) {
    LOG_BINARY_SENSOR("  ", "Balancing", sensor);
  }
#endif
}

bool BQ769X0Component::read_registers_(uint8_t a_register, uint8_t *data, size_t len) {
  if (len > MAX_READ_LEN) {
    return false;
  }
  if (!this->crc_enabled_) {
    return this->read_register(a_register, data, len) == i2c::ERROR_OK;
  }

  // CRC parts follow every data byte with its CRC.
  uint8_t buf[MAX_READ_LEN * 2];
  if (this->read_register(a_register, buf, len * 2) != i2c::ERROR_OK) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    const uint8_t value = buf[i * 2];
    const uint8_t expected = read_crc(this->address_, i, value);
    if (expected != buf[i * 2 + 1]) {
      ESP_LOGW(TAG, "CRC mismatch reading register 0x%02X: data 0x%02X, CRC 0x%02X, expected 0x%02X", a_register + i,
               value, buf[i * 2 + 1], expected);
      return false;
    }
    data[i] = value;
  }
  return true;
}

bool BQ769X0Component::write_register_(uint8_t a_register, uint8_t value) {
  if (!this->crc_enabled_) {
    return this->write_register(a_register, &value, 1) == i2c::ERROR_OK;
  }
  const uint8_t payload[2] = {value, write_crc(this->address_, a_register, value)};
  return this->write_register(a_register, payload, sizeof(payload)) == i2c::ERROR_OK;
}

void BQ769X0Component::wake_device_() {
  this->wakeup_pin_->pin_mode(gpio::FLAG_OUTPUT);
  this->wakeup_pin_->digital_write(true);
  delay(WAKE_PULSE_MS);
  this->wakeup_pin_->digital_write(false);
  // Release TS1 so the thermistor input is not loaded by the GPIO.
  this->wakeup_pin_->pin_mode(gpio::FLAG_INPUT);
  delay(BOOT_DELAY_MS);
}

void BQ769X0Component::handle_alert_() {
  uint8_t status;
  if (!this->read_register_(reg::SYS_STAT, &status)) {
    ESP_LOGW(TAG, "Failed to read SYS_STAT");
    return;
  }
  ESP_LOGVV(TAG, "SYS_STAT: 0x%02X", status);
  // CC_READY fires every 250 ms while the coulomb counter runs, anything else is worth reporting.
  if (status & ~sys_stat::CC_READY) {
    ESP_LOGW(TAG, "SYS_STAT alert: 0x%02X", status);
  }
  // Status bits are cleared by writing 1 to them.
  if (status != 0 && !this->write_register_(reg::SYS_STAT, status)) {
    ESP_LOGW(TAG, "Failed to clear SYS_STAT");
  }
}

}  // namespace esphome::bq769x0
