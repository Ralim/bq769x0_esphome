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

  // The device resets these on every power-on, so they are always rewritten. Configured before the ADC is enabled so
  // OV/UV protection never runs on the defaults.
  if (!this->configure_protection_()) {
    ESP_LOGE(TAG, "Failed to configure hardware protection");
    this->mark_failed();
    return;
  }
  // Faults latched before boot were judged against the previous protection settings.
  uint8_t status;
  if (!this->read_register_(reg::SYS_STAT, &status) || (status != 0 && !this->write_register_(reg::SYS_STAT, status))) {
    ESP_LOGE(TAG, "Failed to clear SYS_STAT");
    this->mark_failed();
    return;
  }
  if (status & static_cast<uint8_t>(~sys_stat::CC_READY)) {
    ESP_LOGI(TAG, "Cleared status latched before boot: 0x%02X", status);
  }

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

bool BQ769X0Component::configure_protection_() {
  // OCD/SCD thresholds are voltages across the shunt; without one only the lowest thresholds can be used.
  const float to_mv = this->shunt_resistance_ * 1000.0f;
  const float ocd_mv = this->max_discharge_a_ * to_mv;
  const float scd_mv = this->short_circuit_a_ * to_mv;
  const bool upper = protection_upper_range(ocd_mv, scd_mv);
  const uint8_t ocd_code = threshold_code_at_or_above(OCD_THRESHOLDS_MV[upper], 16, ocd_mv);
  const uint8_t scd_code = threshold_code_at_or_above(SCD_THRESHOLDS_MV[upper], 8, scd_mv);

  // Hardware OV/UV are a backstop just beyond the software limits, so the software always acts first.
  // Without a configured limit the device default is kept. Anything more than one step away was clamped.
  const int32_t step_mv = (16 * this->adc_gain_uv_ + 999) / 1000;
  uint8_t ov_trip = OV_TRIP_DEFAULT;
  uint8_t uv_trip = UV_TRIP_DEFAULT;
  if (this->max_cell_mv_ > 0) {
    ov_trip = ov_trip_code(this->max_cell_mv_, this->adc_gain_uv_, this->adc_offset_mv_);
    const int32_t actual = ov_trip_mv(ov_trip, this->adc_gain_uv_, this->adc_offset_mv_);
    if (actual < this->max_cell_mv_ || actual - this->max_cell_mv_ > step_mv) {
      ESP_LOGW(TAG, "maximum_cell_voltage is outside the hardware OV range, using %" PRId32 " mV", actual);
    }
  }
  if (this->min_cell_mv_ > 0) {
    uv_trip = uv_trip_code(this->min_cell_mv_, this->adc_gain_uv_, this->adc_offset_mv_);
    const int32_t actual = uv_trip_mv(uv_trip, this->adc_gain_uv_, this->adc_offset_mv_);
    if (actual > this->min_cell_mv_ || this->min_cell_mv_ - actual > step_mv) {
      ESP_LOGW(TAG, "minimum_cell_voltage is outside the hardware UV range, using %" PRId32 " mV", actual);
    }
  }

  this->protect_[0] = protect1_value(upper, this->scd_delay_code_, scd_code);
  this->protect_[1] = protect2_value(this->ocd_delay_code_, ocd_code);
  this->protect_[2] = protect3_value(this->uv_delay_code_, this->ov_delay_code_);
  this->protect_[3] = ov_trip;
  this->protect_[4] = uv_trip;

  // PROTECT1..UV_TRIP are contiguous.
  for (uint8_t i = 0; i < sizeof(this->protect_); i++) {
    if (!this->write_register_(reg::PROTECT1 + i, this->protect_[i])) {
      return false;
    }
  }
  uint8_t readback[sizeof(this->protect_)];
  if (!this->read_registers_(reg::PROTECT1, readback, sizeof(readback))) {
    return false;
  }
  for (uint8_t i = 0; i < sizeof(this->protect_); i++) {
    // PROTECT1 bit 6 and PROTECT2 bit 7 are read-only.
    const uint8_t mask = i == 0 ? 0xBF : (i == 1 ? 0x7F : 0xFF);
    if ((readback[i] & mask) != this->protect_[i]) {
      ESP_LOGE(TAG, "Protection register 0x%02X reads 0x%02X, wrote 0x%02X", reg::PROTECT1 + i, readback[i],
               this->protect_[i]);
      return false;
    }
  }
  return true;
}

void BQ769X0Component::loop() {
  if (this->alert_pin_ != nullptr && this->alert_pin_->digital_read()) {
    this->process_status_();
  }
}

void BQ769X0Component::update() {
  // Also polled here so faults are cleared and latched without an alert pin.
  bool ok = this->process_status_();
  ok &= this->update_cells_();
  ok &= this->update_pack_voltage_();
  ok &= this->update_current_();
  if (this->balancing_enabled_) {
    ok &= this->run_balancing_();
  }
  ok &= this->update_balancing_();
  ok &= this->update_fets_();
  this->publish_alarms_();
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
  if (this->max_cell_delta_sensor_ != nullptr) {
    this->max_cell_delta_sensor_->publish_state(valid ? max_cell_delta_mv(this->input_mv_, mask, inputs) : NAN);
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

bool BQ769X0Component::read_current_() {
  uint8_t data[2];
  if (!this->read_registers_(reg::CC_HI, data, sizeof(data))) {
    ESP_LOGW(TAG, "Failed to read pack current");
    this->current_valid_ = false;
    return false;
  }
  const auto raw = static_cast<int16_t>((data[0] << 8) | data[1]);
  this->current_a_ = cc_current_a(raw, this->shunt_resistance_);
  this->current_valid_ = true;
  ESP_LOGVV(TAG, "Current: %.3f A (CC raw %d)", this->current_a_, raw);
  return true;
}

bool BQ769X0Component::update_current_() {
  if (this->shunt_resistance_ <= 0.0f) {
    return true;
  }
  // process_status_() keeps current_a_ up to date from every fresh coulomb counter reading.
  if (this->current_valid_) {
    ESP_LOGD(TAG, "Current: %.3f A", this->current_a_);
  }
#ifdef USE_SENSOR
  if (this->current_sensor_ != nullptr) {
    this->current_sensor_->publish_state(this->current_valid_ ? this->current_a_ : NAN);
  }
#endif
  return this->current_valid_;
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

uint8_t BQ769X0Component::controlled_fets_() const {
  uint8_t fets = 0;
  if (this->min_cell_mv_ > 0 || this->max_discharge_a_ > 0.0f) {
    fets |= sys_ctrl2::DSG_ON;
  }
  if (this->max_cell_mv_ > 0 || this->max_charge_a_ > 0.0f) {
    fets |= sys_ctrl2::CHG_ON;
  }
  return fets;
}

bool BQ769X0Component::check_current_limits_() {
  const uint8_t blocked =
      this->current_valid_ ? fets_blocked_by_current(this->current_a_, this->max_charge_a_, this->max_discharge_a_) : 0;
  // Readings over the limit while a FET is already held off don't extend the hold, so the retry always happens
  // `overcurrent_recovery_ms_` after the trip.
  const uint8_t tripped = blocked & static_cast<uint8_t>(~this->overcurrent_fets_);
  if (tripped & sys_ctrl2::CHG_ON) {
    ESP_LOGW(TAG, "Charge current %.3f A exceeds %.3f A, disabling charge for %" PRIu32 " ms", this->current_a_,
             this->max_charge_a_, this->overcurrent_recovery_ms_);
    this->schedule_overcurrent_retry_(sys_ctrl2::CHG_ON);
  }
  if (tripped & sys_ctrl2::DSG_ON) {
    ESP_LOGW(TAG, "Discharge current %.3f A exceeds %.3f A, disabling discharge for %" PRIu32 " ms", -this->current_a_,
             this->max_discharge_a_, this->overcurrent_recovery_ms_);
    this->schedule_overcurrent_retry_(sys_ctrl2::DSG_ON);
  }
  this->overcurrent_fets_ |= tripped;
  return tripped != 0;
}

void BQ769X0Component::schedule_overcurrent_retry_(uint8_t fet) {
  const bool charge = fet == sys_ctrl2::CHG_ON;
  this->set_timeout(charge ? "charge_oc_retry" : "discharge_oc_retry", this->overcurrent_recovery_ms_,
                    [this, fet, charge]() {
                      ESP_LOGI(TAG, "Retrying %s after overcurrent", charge ? "charge" : "discharge");
                      this->overcurrent_fets_ &= static_cast<uint8_t>(~fet);
                      // A still-excessive current trips the FET off again on the next reading.
                      this->update_fets_();
                      this->publish_alarms_();
                    });
}

bool BQ769X0Component::update_fets_() {
  const uint8_t controlled = this->controlled_fets_();
  uint8_t sys_ctrl1, sys_ctrl2;
  if (!this->read_register_(reg::SYS_CTRL1, &sys_ctrl1) || !this->read_register_(reg::SYS_CTRL2, &sys_ctrl2)) {
    ESP_LOGW(TAG, "Failed to read FET state");
    return false;
  }
  if (controlled == 0) {
    this->publish_fet_state_(sys_ctrl2);
    return true;
  }

  uint8_t value;
  if (this->load_fault_) {
    // LOAD_PRESENT is only valid while CHG is off, which is guaranteed after the first cycle of the fault.
    if (!(sys_ctrl2 & sys_ctrl2::CHG_ON) && !(sys_ctrl1 & sys_ctrl1::LOAD_PRESENT)) {
      ESP_LOGI(TAG, "Load removed, clearing discharge fault");
      this->load_fault_ = false;
    }
  }
  if (this->load_fault_) {
    value = sys_ctrl2 & static_cast<uint8_t>(~(sys_ctrl2::CHG_ON | sys_ctrl2::DSG_ON));
  } else {
    const uint8_t inputs = max_cells(this->model_);
    // Without trustworthy cell readings every controlled FET is turned off.
    const uint8_t on = this->cells_valid_
                           ? fets_allowed_by_voltage(lowest_cell_mv(this->input_mv_, this->cell_inputs_, inputs),
                                                     highest_cell_mv(this->input_mv_, this->cell_inputs_, inputs),
                                                     this->min_cell_mv_, this->max_cell_mv_)
                           : 0;
    value = apply_fets(sys_ctrl2, controlled, on & static_cast<uint8_t>(~this->overcurrent_fets_));
  }
  if (value == sys_ctrl2) {
    this->publish_fet_state_(sys_ctrl2);
    return true;
  }
  ESP_LOGI(TAG, "FETs: charge %s, discharge %s", ONOFF(value & sys_ctrl2::CHG_ON), ONOFF(value & sys_ctrl2::DSG_ON));
  if (!this->write_register_(reg::SYS_CTRL2, value)) {
    ESP_LOGW(TAG, "Failed to write FET state");
    this->publish_fet_state_(sys_ctrl2);
    return false;
  }
  this->publish_fet_state_(value);
  return true;
}

void BQ769X0Component::publish_fet_state_(uint8_t sys_ctrl2) {
#ifdef USE_BINARY_SENSOR
  if (this->charge_fet_sensor_ != nullptr) {
    this->charge_fet_sensor_->publish_state(sys_ctrl2 & sys_ctrl2::CHG_ON);
  }
  if (this->discharge_fet_sensor_ != nullptr) {
    this->discharge_fet_sensor_->publish_state(sys_ctrl2 & sys_ctrl2::DSG_ON);
  }
#endif
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
  if (this->min_cell_mv_ != 0) {
    ESP_LOGCONFIG(TAG, "  Minimum cell voltage: %" PRId32 " mV", this->min_cell_mv_);
  }
  if (this->max_cell_mv_ != 0) {
    ESP_LOGCONFIG(TAG, "  Maximum cell voltage: %" PRId32 " mV", this->max_cell_mv_);
  }
  if (this->max_charge_a_ > 0.0f) {
    ESP_LOGCONFIG(TAG, "  Maximum charge current: %.3f A", this->max_charge_a_);
  }
  if (this->max_discharge_a_ > 0.0f) {
    ESP_LOGCONFIG(TAG, "  Maximum discharge current: %.3f A", this->max_discharge_a_);
  }
  if (this->max_charge_a_ > 0.0f || this->max_discharge_a_ > 0.0f) {
    ESP_LOGCONFIG(TAG, "  Overcurrent recovery time: %" PRIu32 " ms", this->overcurrent_recovery_ms_);
  }
  {
    const bool upper = this->protect_[0] & 0x80;
    const uint8_t scd_mv = SCD_THRESHOLDS_MV[upper][this->protect_[0] & 0x07];
    const uint8_t ocd_mv = OCD_THRESHOLDS_MV[upper][this->protect_[1] & 0x0F];
    const float to_a = this->shunt_resistance_ > 0.0f ? 0.001f / this->shunt_resistance_ : NAN;
    ESP_LOGCONFIG(TAG,
                  "  Hardware protection:\n"
                  "    OV: %" PRId32 " mV after %u s\n"
                  "    UV: %" PRId32 " mV after %u s\n"
                  "    OCD: %u mV (%.2f A) after %u ms\n"
                  "    SCD: %u mV (%.2f A) after %u us",
                  ov_trip_mv(this->protect_[3], this->adc_gain_uv_, this->adc_offset_mv_),
                  OV_DELAYS_S[(this->protect_[2] >> 4) & 0x03],
                  uv_trip_mv(this->protect_[4], this->adc_gain_uv_, this->adc_offset_mv_),
                  UV_DELAYS_S[(this->protect_[2] >> 6) & 0x03], ocd_mv, ocd_mv * to_a,
                  OCD_DELAYS_MS[(this->protect_[1] >> 4) & 0x07], scd_mv, scd_mv * to_a,
                  SCD_DELAYS_US[(this->protect_[0] >> 3) & 0x03]);
  }
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
  LOG_SENSOR("  ", "Max Cell Delta", this->max_cell_delta_sensor_);
#endif
#ifdef USE_BINARY_SENSOR
  for (auto *sensor : this->balancing_sensors_) {
    LOG_BINARY_SENSOR("  ", "Balancing", sensor);
  }
  LOG_BINARY_SENSOR("  ", "Charge FET", this->charge_fet_sensor_);
  LOG_BINARY_SENSOR("  ", "Discharge FET", this->discharge_fet_sensor_);
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Alarms", this->alarms_sensor_);
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

bool BQ769X0Component::process_status_() {
  uint8_t status;
  if (!this->read_register_(reg::SYS_STAT, &status)) {
    ESP_LOGW(TAG, "Failed to read SYS_STAT");
    return false;
  }
  ESP_LOGVV(TAG, "SYS_STAT: 0x%02X", status);
  // CC_READY fires every 250 ms while the coulomb counter runs, anything else is worth reporting.
  if (status & ~sys_stat::CC_READY) {
    ESP_LOGV(TAG, "SYS_STAT alert: 0x%02X", status);
  }
  if (status & (sys_stat::DEVICE_XREADY | sys_stat::OVRD_ALERT)) {
    ESP_LOGW(TAG, "SYS_STAT alert: 0x%02X", status);
  }
  // The device has already turned DSG off. Keep it off until the load is gone rather than switching it back on
  // into a short.
  bool short_circuit = status & sys_stat::SCD;
  if (status & sys_stat::OCD) {
    if (this->max_discharge_a_ > 0.0f) {
      // Hardware OCD enforces maximum_discharge_current, so it gets the same timed retry as the software limit.
      if (!(this->overcurrent_fets_ & sys_ctrl2::DSG_ON)) {
        ESP_LOGW(TAG, "Hardware discharge overcurrent, disabling discharge for %" PRIu32 " ms",
                 this->overcurrent_recovery_ms_);
        this->overcurrent_fets_ |= sys_ctrl2::DSG_ON;
        this->schedule_overcurrent_retry_(sys_ctrl2::DSG_ON);
      }
    } else {
      short_circuit = true;
    }
  }
  if (short_circuit && !this->load_fault_) {
    ESP_LOGW(TAG, "Short circuit, FETs held off until the load is removed");
    this->load_fault_ = true;
  }
  this->track_voltage_trip_(0, status & sys_stat::OV);
  this->track_voltage_trip_(1, status & sys_stat::UV);
  // A fresh coulomb counter reading every 250 ms; reading it here keeps overcurrent response fast when the
  // alert pin is used.
  bool ok = true;
  if ((status & sys_stat::CC_READY) && this->shunt_resistance_ > 0.0f) {
    ok = this->read_current_();
    if (this->check_current_limits_()) {
      ok &= this->update_fets_();
    }
  }
  // Status bits are cleared by writing 1 to them; the device sets them again if the fault persists.
  if (status != 0 && !this->write_register_(reg::SYS_STAT, status)) {
    ESP_LOGW(TAG, "Failed to clear SYS_STAT");
    return false;
  }
  this->publish_alarms_();
  return ok;
}

void BQ769X0Component::track_voltage_trip_(uint8_t index, bool tripped) {
  static const char *const NAMES[2] = {"overvoltage", "undervoltage"};
  static const char *const FETS[2] = {"charge", "discharge"};
  const uint32_t now = millis();
  // While the fault persists the device trips again one delay after each clear; a gap longer than that ends it.
  const uint32_t delay_s = index == 0 ? OV_DELAYS_S[this->ov_delay_code_] : UV_DELAYS_S[this->uv_delay_code_];
  const uint32_t timeout_ms = delay_s * 1000 + VOLTAGE_TRIP_MARGIN_MS;
  if (tripped) {
    if (!this->voltage_trip_active_[index]) {
      ESP_LOGW(TAG, "Hardware %s tripped, %s disabled", NAMES[index], FETS[index]);
      this->voltage_trip_active_[index] = true;
    }
    this->voltage_trip_ms_[index] = now;
  } else if (this->voltage_trip_active_[index] && now - this->voltage_trip_ms_[index] > timeout_ms) {
    ESP_LOGI(TAG, "Hardware %s cleared", NAMES[index]);
    this->voltage_trip_active_[index] = false;
  }
}

uint8_t BQ769X0Component::active_alarms_() const {
  uint8_t alarms = 0;
  if (this->cells_valid_) {
    const uint8_t inputs = max_cells(this->model_);
    alarms |= voltage_alarms(lowest_cell_mv(this->input_mv_, this->cell_inputs_, inputs),
                             highest_cell_mv(this->input_mv_, this->cell_inputs_, inputs), this->min_cell_mv_,
                             this->max_cell_mv_);
  }
  if (this->overcurrent_fets_ & sys_ctrl2::CHG_ON) {
    alarms |= alarm::OVER_CHARGE_CURRENT;
  }
  if (this->overcurrent_fets_ & sys_ctrl2::DSG_ON) {
    alarms |= alarm::OVER_DISCHARGE_CURRENT;
  }
  if (this->load_fault_) {
    alarms |= alarm::SHORT_CIRCUIT;
  }
  return alarms;
}

void BQ769X0Component::publish_alarms_() {
#ifdef USE_TEXT_SENSOR
  if (this->alarms_sensor_ == nullptr) {
    return;
  }
  const uint8_t alarms = this->active_alarms_();
  if (alarms == this->published_alarms_) {
    return;
  }
  char text[alarms_text_size()];
  format_alarms(alarms, text, sizeof(text));
  if (alarms != 0) {
    ESP_LOGW(TAG, "Alarms: %s", text);
  } else if (this->published_alarms_ != 0xFF) {
    ESP_LOGI(TAG, "Alarms cleared");
  }
  this->alarms_sensor_->publish_state(text);
  this->published_alarms_ = alarms;
#endif
}

}  // namespace esphome::bq769x0
