#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"
#include "esphome/core/time.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>

namespace esphome {
namespace apaphx_ads1015 {

static const char *const TAG = "apaphx.ads1015";

// ADS1015 registers and constants
static const uint8_t ADS1015_REG_POINTER_CONVERT = 0x00;
static const uint8_t ADS1015_REG_POINTER_CONFIG = 0x01;
static const uint16_t ADS1015_REG_CONFIG_OS_SINGLE = 0x8000;
static const uint16_t ADS1015_REG_CONFIG_MUX_SINGLE_0 = 0x4000;
static const uint16_t ADS1015_REG_CONFIG_MODE_CONTIN = 0x0000;
static const uint16_t ADS1015_REG_CONFIG_DR_1600SPS = 0x0080;

// Gain settings
static const uint16_t ADS1015_REG_CONFIG_PGA_6_144V = 0x0000;
static const uint16_t ADS1015_REG_CONFIG_PGA_4_096V = 0x0200;
static const uint16_t ADS1015_REG_CONFIG_PGA_2_048V = 0x0400;
static const uint16_t ADS1015_REG_CONFIG_PGA_1_024V = 0x0600;
static const uint16_t ADS1015_REG_CONFIG_PGA_0_512V = 0x0800;
static const uint16_t ADS1015_REG_CONFIG_PGA_0_256V = 0x0A00;

// Stable reading configuration
static const uint8_t SAMPLES_PER_PORTION = 100;   // Number of samples per portion
static const uint32_t SAMPLE_DELAY_MS = 10;       // Delay between samples
static const uint32_t PORTION_DELAY_MS = 500;     // Delay between portions
static const float STABILITY_THRESHOLD = 0.5f;     // Maximum allowed difference (mV)
static const uint8_t MAX_ATTEMPTS = 60;           // Maximum number of attempts

// Temperature compensation constants for chlorinated pools
static constexpr float REFERENCE_TEMP_C = 25.0f;  // Standard pH calibration temperature
static constexpr float CHLORINE_PH_TEMP_COEF = 0.034f;  // Updated coefficient for chlorinated water
static constexpr float NEUTRAL_PH = 7.0f;  // Reference point for compensation calculation

// Pump stabilization time
static constexpr uint32_t PUMP_STABILIZE_TIME = 30000;  // 30 seconds in ms

// Structure for temperature compensation logging
struct TempCompensationLog {
    float raw_ph;
    float compensated_ph;
    float temperature;
    time_t timestamp;
};

// Circular buffer template for efficient logging
template<typename T, size_t S>
class CircularBuffer {
public:
    void push(const T& item) {
        buffer_[write_index_] = item;
        write_index_ = (write_index_ + 1) % S;
        if (count_ < S) count_++;
    }
    
    const T* get_buffer() const { return buffer_; }
    size_t get_count() const { return count_; }
    
private:
    T buffer_[S];
    size_t write_index_ = 0;
    size_t count_ = 0;
};

struct CalibrationData {
    float ph_ref1_mv;
    float ph_ref2_mv;
    float ph_ref1_value;
    float ph_ref2_value;
    uint32_t ph_last_calibration;  // Unix timestamp of last pH calibration
    
    float orp_ref1_mv;
    float orp_ref2_mv;
    float orp_ref1_value;
    float orp_ref2_value;
    uint32_t orp_last_calibration;  // Unix timestamp of last ORP calibration
};

class APAPHX_ADS1015 : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
    APAPHX_ADS1015();

    void setup() override;
    void dump_config() override;
    void update() override;
    
    // Basic reading functions
    float read_voltage(uint8_t address, uint8_t channel, uint16_t gain);
    float convert_to_ph(float voltage);
    float convert_to_orp(float voltage);
    
    // Stable reading function
    float get_stable_reading(uint8_t address, uint16_t gain);
    
    // Configuration setters
    void set_ph_gain(uint16_t gain) { ph_gain_ = gain; }
    void set_orp_gain(uint16_t gain) { orp_gain_ = gain; }
    void set_ph_address(uint8_t address) { ph_address_ = address; }
    void set_orp_address(uint8_t address) { orp_address_ = address; }
    void set_ph_sensor(sensor::Sensor *sens) { ph_sensor_ = sens; }
    void set_orp_sensor(sensor::Sensor *sens) { orp_sensor_ = sens; }
    void set_orp_voltage_sensor(sensor::Sensor *sens) { orp_voltage_sensor_ = sens; }
    void set_ph_cal_age_sensor(sensor::Sensor *sens) { ph_cal_age_sensor_ = sens; }
    void set_orp_cal_age_sensor(sensor::Sensor *sens) { orp_cal_age_sensor_ = sens; }
    void set_temperature_sensor(sensor::Sensor *temp_sensor) { temp_sensor_ = temp_sensor; }
    void set_pump_sensor(binary_sensor::BinarySensor *pump_sensor) { pump_sensor_ = pump_sensor; }
    void set_ema_alpha(float alpha) { ema_alpha_ = alpha; }
    void set_median_window(uint8_t window) {
        median_window_ = window;
        ph_buffer_.resize(window);
        orp_buffer_.resize(window);
    }

    // Calibration functions
    void calibrate_ph_point1(float ph_value);
    void calibrate_ph_point2(float ph_value);
    void calibrate_orp_point1(float mv_value);
    void calibrate_orp_point2(float mv_value);
    void reset_calibration();

    // Temperature compensation log access
    const TempCompensationLog* get_temp_comp_log() const { 
        return temp_comp_log_.get_buffer(); 
    }
    size_t get_temp_comp_log_count() const { 
        return temp_comp_log_.get_count(); 
    }

 protected:
    void load_calibration_();
    void save_calibration_();
    void update_calibration_age_();
    
    // Filtering helper functions
    float apply_ema_(float new_value, float& ema_value);
    float apply_median_filter_(std::vector<float>& buffer, float new_value);
    float get_filtered_reading_(uint8_t address, uint8_t channel, uint16_t gain);
    float compensate_ph_for_temperature_(float ph, float temp_c);
    void report_memory_status_();

    // Member variables
    uint16_t ph_gain_{ADS1015_REG_CONFIG_PGA_4_096V};
    uint16_t orp_gain_{ADS1015_REG_CONFIG_PGA_4_096V};
    uint8_t ph_address_{0x49};
    uint8_t orp_address_{0x48};
    sensor::Sensor *ph_sensor_{nullptr};
    sensor::Sensor *orp_sensor_{nullptr};
    sensor::Sensor *orp_voltage_sensor_{nullptr};
    sensor::Sensor *ph_cal_age_sensor_{nullptr};
    sensor::Sensor *orp_cal_age_sensor_{nullptr};
    sensor::Sensor *temp_sensor_{nullptr};
    binary_sensor::BinarySensor *pump_sensor_{nullptr};
    ESPPreferenceObject pref_;
    CalibrationData cal_data_{0, 0, 4.0f, 7.0f, 0, 0, 0, 475.0f, 650.0f, 0};

    // Filtering variables
    float ema_alpha_{0.1f};  // Default EMA smoothing factor
    uint8_t median_window_{5};  // Default median window size
    std::vector<float> ph_buffer_;
    std::vector<float> orp_buffer_;
    float ph_ema_{NAN};
    float orp_ema_{NAN};

    // Memory optimization
    uint8_t read_attempt_count_{0};
    uint8_t calibration_count_{0};
    
    // Temperature compensation logging
    static const size_t TEMP_COMP_LOG_SIZE = 24;  // Store last 24 readings
    CircularBuffer<TempCompensationLog, TEMP_COMP_LOG_SIZE> temp_comp_log_;

    // Pump control variables
    bool was_pump_running_{false};
    uint32_t pump_start_time_{0};

    // Status flags
    struct StatusFlags {
        uint8_t reading_valid : 1;
        uint8_t temp_comp_enabled : 1;
        uint8_t calibration_valid : 1;
        uint8_t sensor_initialized : 1;
        uint8_t reserved : 4;  // For future use
    } status_flags_{};
};

// Action classes defined after APAPHX_ADS1015 is complete
template<typename... Ts> class CalibratePh1Action : public Action<Ts...> {
 public:
  explicit CalibratePh1Action(APAPHX_ADS1015 *parent, float value) : parent_(parent), value_(value) {}
  void play(Ts... x) override {
    this->parent_->calibrate_ph_point1(this->value_);
  }
 protected:
  APAPHX_ADS1015 *parent_;
  float value_;
};

template<typename... Ts> class CalibratePh2Action : public Action<Ts...> {
 public:
  explicit CalibratePh2Action(APAPHX_ADS1015 *parent, float value) : parent_(parent), value_(value) {}
  void play(Ts... x) override {
    this->parent_->calibrate_ph_point2(this->value_);
  }
 protected:
  APAPHX_ADS1015 *parent_;
  float value_;
};

template<typename... Ts> class CalibrateOrp1Action : public Action<Ts...> {
 public:
  explicit CalibrateOrp1Action(APAPHX_ADS1015 *parent, float value) : parent_(parent), value_(value) {}
  void play(Ts... x) override {
    this->parent_->calibrate_orp_point1(this->value_);
  }
 protected:
  APAPHX_ADS1015 *parent_;
  float value_;
};

template<typename... Ts> class CalibrateOrp2Action : public Action<Ts...> {
 public:
  explicit CalibrateOrp2Action(APAPHX_ADS1015 *parent, float value) : parent_(parent), value_(value) {}
  void play(Ts... x) override {
    this->parent_->calibrate_orp_point2(this->value_);
  }
 protected:
  APAPHX_ADS1015 *parent_;
  float value_;
};

template<typename... Ts> class ResetCalibrationAction : public Action<Ts...> {
 public:
  explicit ResetCalibrationAction(APAPHX_ADS1015 *parent) : parent_(parent) {}
  void play(Ts... x) override {
    this->parent_->reset_calibration();
  }
 protected:
  APAPHX_ADS1015 *parent_;
};

}  // namespace apaphx_ads1015
}  // namespace esphome