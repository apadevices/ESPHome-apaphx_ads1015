#include "apaphx_ads1015.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"
#include <algorithm>

namespace esphome {
namespace apaphx_ads1015 {

APAPHX_ADS1015::APAPHX_ADS1015() 
    : sensor::Sensor(), PollingComponent(5000), i2c::I2CDevice() {
    ph_buffer_.resize(median_window_);
    orp_buffer_.resize(median_window_);
}

void APAPHX_ADS1015::setup() {
    ESP_LOGCONFIG(TAG, "Setting up APA PHX ADS1015...");
    this->pref_ = global_preferences->make_preference<CalibrationData>(879452);
    load_calibration_();

    // Initialize status flags
    status_flags_.reading_valid = false;
    status_flags_.temp_comp_enabled = (temp_sensor_ != nullptr);
    status_flags_.calibration_valid = false;
    status_flags_.sensor_initialized = false;

    // Test both ADCs
    this->set_i2c_address(ph_address_);
    if (!this->read_byte(ADS1015_REG_POINTER_CONFIG)) {
        this->mark_failed();
        return;
    }

    this->set_i2c_address(orp_address_);
    if (!this->read_byte(ADS1015_REG_POINTER_CONFIG)) {
        this->mark_failed();
        return;
    }

    status_flags_.sensor_initialized = true;
    report_memory_status_();
}

void APAPHX_ADS1015::dump_config() {
    ESP_LOGCONFIG(TAG, "APA PHX ADS1015:");
    ESP_LOGCONFIG(TAG, "  pH ADC Address: 0x%02X", this->ph_address_);
    ESP_LOGCONFIG(TAG, "  ORP ADC Address: 0x%02X", this->orp_address_);
    ESP_LOGCONFIG(TAG, "  EMA Alpha: %.3f", this->ema_alpha_);
    ESP_LOGCONFIG(TAG, "  Median Window: %d", this->median_window_);
    ESP_LOGCONFIG(TAG, "  Temperature Compensation: %s", 
                 this->temp_sensor_ != nullptr ? "Yes" : "No");
    ESP_LOGCONFIG(TAG, "  Pump Control: %s",
                 this->pump_sensor_ != nullptr ? "Yes" : "No");
    
    time_t now = ::time(nullptr);
    if (cal_data_.ph_last_calibration > 0) {
        ESP_LOGCONFIG(TAG, "  pH Calibration: %u days old", 
                     (now - cal_data_.ph_last_calibration) / 86400);
    }
    if (cal_data_.orp_last_calibration > 0) {
        ESP_LOGCONFIG(TAG, "  ORP Calibration: %u days old", 
                     (now - cal_data_.orp_last_calibration) / 86400);
    }
    LOG_I2C_DEVICE(this);
    if (this->is_failed()) {
        ESP_LOGE(TAG, "Communication failed!");
    }
}

void APAPHX_ADS1015::report_memory_status_() {
    ESP_LOGD(TAG, "Memory Status:");
    ESP_LOGD(TAG, "  Temp Comp Log: %u entries (max %u)", 
             temp_comp_log_.get_count(), TEMP_COMP_LOG_SIZE);
    ESP_LOGD(TAG, "  pH Buffer Size: %u samples", ph_buffer_.size());
    ESP_LOGD(TAG, "  ORP Buffer Size: %u samples", orp_buffer_.size());
}

float APAPHX_ADS1015::apply_ema_(float new_value, float& ema_value) {
    if (std::isnan(ema_value)) {
        ema_value = new_value;
    } else {
        ema_value = (ema_alpha_ * new_value) + ((1.0f - ema_alpha_) * ema_value);
    }
    return ema_value;
}

float APAPHX_ADS1015::apply_median_filter_(std::vector<float>& buffer, float new_value) {
    static size_t buffer_index = 0;
    
    // Ensure buffer is properly sized
    if (buffer.size() != median_window_) {
        buffer.resize(median_window_);
    }
    
    buffer[buffer_index] = new_value;
    buffer_index = (buffer_index + 1) % median_window_;
    
    // Create temporary vector for sorting
    std::vector<float> sorted = buffer;
    std::sort(sorted.begin(), sorted.end());
    
    return sorted[median_window_ / 2];
}

float APAPHX_ADS1015::get_filtered_reading_(uint8_t address, uint8_t channel, uint16_t gain) {
    float raw_value = read_voltage(address, channel, gain);
    if (std::isnan(raw_value)) return NAN;
    
    // Apply median filter
    auto& buffer = (address == ph_address_) ? ph_buffer_ : orp_buffer_;
    float median_value = apply_median_filter_(buffer, raw_value);
    
    // Apply EMA
    float& ema_value = (address == ph_address_) ? ph_ema_ : orp_ema_;
    return apply_ema_(median_value, ema_value);
}

float APAPHX_ADS1015::compensate_ph_for_temperature_(float ph_25c, float temp_c) {
    // Modified Nernst equation for chlorinated pool water
    float temp_diff = temp_c - REFERENCE_TEMP_C;
    float ph_factor = (NEUTRAL_PH - ph_25c) / NEUTRAL_PH;
    float compensation = CHLORINE_PH_TEMP_COEF * temp_diff * ph_factor;
    float compensated_ph = ph_25c + compensation;
    
    // Add range limiting
    compensated_ph = std::min(std::max(compensated_ph, 0.0f), 14.0f);
    
    // Log compensation details for analysis
    TempCompensationLog log_entry = {
        .raw_ph = ph_25c,
        .compensated_ph = compensated_ph,
        .temperature = temp_c,
        .timestamp = ::time(nullptr)
    };
    temp_comp_log_.push(log_entry);
    
    // Add enhanced logging for significant changes
    float compensation_magnitude = abs(compensated_ph - ph_25c);
    if (compensation_magnitude > 0.2f) {  // Significant compensation threshold
        ESP_LOGW(TAG, "Large pH temperature compensation: %.2f pH units at %.1f°C", 
                compensation_magnitude, temp_c);
    }
    
    return compensated_ph;
}

float APAPHX_ADS1015::get_stable_reading(uint8_t address, uint16_t gain) {
    float firstReading, secondReading;
    int attempts = 0;
    
    do {
        // First portion of readings
        float sum = 0;
        int valid = 0;
        
        for (int i = 0; i < SAMPLES_PER_PORTION; i++) {
            float reading = get_filtered_reading_(address, 0, gain);
            if (!std::isnan(reading)) {
                sum += reading;
                valid++;
            }
            delay(SAMPLE_DELAY_MS);
        }
        
        if (valid == 0) return NAN;
        firstReading = sum / valid;
        
        delay(PORTION_DELAY_MS);
        
        // Second portion of readings
        sum = 0;
        valid = 0;
        
        for (int i = 0; i < SAMPLES_PER_PORTION; i++) {
            float reading = get_filtered_reading_(address, 0, gain);
            if (!std::isnan(reading)) {
                sum += reading;
                valid++;
            }
            delay(SAMPLE_DELAY_MS);
        }
        
        if (valid == 0) return NAN;
        secondReading = sum / valid;
        
        float average = (firstReading + secondReading) / 2.0f;
        if (abs(average - firstReading) < STABILITY_THRESHOLD && 
            abs(average - secondReading) < STABILITY_THRESHOLD) {
            return average;
        }
        
        attempts++;
    } while (attempts < MAX_ATTEMPTS);
    
    return NAN;
}

void APAPHX_ADS1015::update() {
    // Check pump status if sensor is connected
    if (pump_sensor_ != nullptr) {
        if (!pump_sensor_->state) {
            if (was_pump_running_) {
                ESP_LOGD(TAG, "Pump stopped - suspending measurements");
                ph_ema_ = NAN;
                orp_ema_ = NAN;
                was_pump_running_ = false;
            }
            return;
        } else if (!was_pump_running_) {
            pump_start_time_ = millis();
            was_pump_running_ = true;
            ESP_LOGD(TAG, "Pump started - waiting 30s before measurements");
            return;
        } else if (millis() - pump_start_time_ < PUMP_STABILIZE_TIME) {
            return;
        }
    }

    // Update pH reading with temperature compensation
    float ph_voltage = get_filtered_reading_(ph_address_, 0, ph_gain_);
    if (!std::isnan(ph_voltage)) {
        float ph_value = convert_to_ph(ph_voltage);
        status_flags_.reading_valid = true;
        
        // Apply temperature compensation if temperature sensor is available
        if (!std::isnan(ph_value) && temp_sensor_ != nullptr && !std::isnan(temp_sensor_->state)) {
            float temp_c = temp_sensor_->state;
            float raw_ph = ph_value;
            ph_value = compensate_ph_for_temperature_(ph_value, temp_c);
            
            ESP_LOGD(TAG, "pH: %.2f (raw) -> %.2f (temp compensated at %.1f°C)", 
                     raw_ph, ph_value, temp_c);
        }
        
        if (!std::isnan(ph_value) && ph_sensor_ != nullptr) {
            ph_sensor_->publish_state(ph_value);
        }
        this->publish_state(ph_voltage);
    } else {
        status_flags_.reading_valid = false;
    }

    // Update ORP reading
    float orp_voltage = get_filtered_reading_(orp_address_, 0, orp_gain_);
    if (!std::isnan(orp_voltage)) {
        // Publish raw ORP voltage if sensor configured
        if (orp_voltage_sensor_ != nullptr) {
            orp_voltage_sensor_->publish_state(orp_voltage);
        }
        
        float orp_value = convert_to_orp(orp_voltage);
        if (!std::isnan(orp_value) && orp_sensor_ != nullptr) {
            orp_sensor_->publish_state(orp_value);
        }
    }

    // Update calibration age sensors
    update_calibration_age_();
}

float APAPHX_ADS1015::read_voltage(uint8_t address, uint8_t channel, uint16_t gain) {
    this->set_i2c_address(address);
    
    uint16_t config = ADS1015_REG_CONFIG_OS_SINGLE |
                     gain |
                     ADS1015_REG_CONFIG_MODE_CONTIN |
                     ADS1015_REG_CONFIG_DR_1600SPS |
                     (ADS1015_REG_CONFIG_MUX_SINGLE_0 + (channel * 0x1000));
    
    if (!write_byte_16(ADS1015_REG_POINTER_CONFIG, config)) {
        return NAN;
    }
    
    delay(1);
    
    uint16_t raw_adc;
    if (!read_byte_16(ADS1015_REG_POINTER_CONVERT, &raw_adc)) {
        return NAN;
    }
    
    raw_adc = raw_adc >> 4;
    
    float voltage_range;
    switch (gain) {
        case ADS1015_REG_CONFIG_PGA_6_144V: voltage_range = 6.144f; break;
        case ADS1015_REG_CONFIG_PGA_4_096V: voltage_range = 4.096f; break;
        case ADS1015_REG_CONFIG_PGA_2_048V: voltage_range = 2.048f; break;
        case ADS1015_REG_CONFIG_PGA_1_024V: voltage_range = 1.024f; break;
        case ADS1015_REG_CONFIG_PGA_0_512V: voltage_range = 0.512f; break;
        case ADS1015_REG_CONFIG_PGA_0_256V: voltage_range = 0.256f; break;
        default: voltage_range = 4.096f;
    }
    
    return raw_adc * voltage_range / 2048.0f;
}

float APAPHX_ADS1015::convert_to_ph(float voltage) {
    float mv = voltage * 1000.0f;
    
    if (abs(cal_data_.ph_ref2_mv - cal_data_.ph_ref1_mv) < 0.001f) {
        return NAN;
    }
    
    float ph = cal_data_.ph_ref1_value + 
              (cal_data_.ph_ref2_value - cal_data_.ph_ref1_value) * 
              (mv - cal_data_.ph_ref1_mv) / 
              (cal_data_.ph_ref2_mv - cal_data_.ph_ref1_mv);
              
    return std::min(std::max(ph, 0.0f), 14.0f);
}

float APAPHX_ADS1015::convert_to_orp(float voltage) {
    float mv = voltage * 1000.0f;
    
    if (abs(cal_data_.orp_ref2_mv - cal_data_.orp_ref1_mv) < 0.001f) {
        return mv;
    }
    
    float orp = cal_data_.orp_ref1_value + 
                (cal_data_.orp_ref2_value - cal_data_.orp_ref1_value) * 
                (mv - cal_data_.orp_ref1_mv) / 
                (cal_data_.orp_ref2_mv - cal_data_.orp_ref1_mv);
                
    return std::min(std::max(orp, -1000.0f), 1000.0f);
}

void APAPHX_ADS1015::load_calibration_() {
    if (!this->pref_.load(&this->cal_data_)) {
        cal_data_ = {0, 0, 4.0f, 7.0f, 0, 0, 0, 475.0f, 650.0f, 0};
    }
    status_flags_.calibration_valid = (cal_data_.ph_last_calibration > 0 || 
                                     cal_data_.orp_last_calibration > 0);
}

void APAPHX_ADS1015::save_calibration_() {
    this->pref_.save(&this->cal_data_);
}

void APAPHX_ADS1015::update_calibration_age_() {
    time_t now = ::time(nullptr);
    
    if (this->ph_cal_age_sensor_ != nullptr && cal_data_.ph_last_calibration > 0) {
        float ph_age_days = (now - cal_data_.ph_last_calibration) / 86400.0f;
        this->ph_cal_age_sensor_->publish_state(ph_age_days);
    }
    
    if (this->orp_cal_age_sensor_ != nullptr && cal_data_.orp_last_calibration > 0) {
        float orp_age_days = (now - cal_data_.orp_last_calibration) / 86400.0f;
        this->orp_cal_age_sensor_->publish_state(orp_age_days);
    }
}

void APAPHX_ADS1015::calibrate_ph_point1(float ph_value) {
    float voltage = get_stable_reading(ph_address_, ph_gain_);
    if (!std::isnan(voltage)) {
        cal_data_.ph_ref1_mv = voltage * 1000.0f;
        cal_data_.ph_ref1_value = ph_value;
        cal_data_.ph_last_calibration = ::time(nullptr);
        save_calibration_();
        update_calibration_age_();
        status_flags_.calibration_valid = true;
        ESP_LOGI(TAG, "pH Point 1 calibrated: %.2f pH at %.1f mV", ph_value, voltage * 1000.0f);
    } else {
        ESP_LOGE(TAG, "pH Point 1 calibration failed: could not get stable reading");
    }
}

void APAPHX_ADS1015::calibrate_ph_point2(float ph_value) {
    float voltage = get_stable_reading(ph_address_, ph_gain_);
    if (!std::isnan(voltage)) {
        cal_data_.ph_ref2_mv = voltage * 1000.0f;
        cal_data_.ph_ref2_value = ph_value;
        cal_data_.ph_last_calibration = ::time(nullptr);
        save_calibration_();
        update_calibration_age_();
        status_flags_.calibration_valid = true;
        ESP_LOGI(TAG, "pH Point 2 calibrated: %.2f pH at %.1f mV", ph_value, voltage * 1000.0f);
    } else {
        ESP_LOGE(TAG, "pH Point 2 calibration failed: could not get stable reading");
    }
}

void APAPHX_ADS1015::calibrate_orp_point1(float mv_value) {
    float voltage = get_stable_reading(orp_address_, orp_gain_);
    if (!std::isnan(voltage)) {
        cal_data_.orp_ref1_mv = voltage * 1000.0f;
        cal_data_.orp_ref1_value = mv_value;
        cal_data_.orp_last_calibration = ::time(nullptr);
        save_calibration_();
        update_calibration_age_();
        status_flags_.calibration_valid = true;
        ESP_LOGI(TAG, "ORP Point 1 calibrated: %.1f mV at %.1f mV", mv_value, voltage * 1000.0f);
    } else {
        ESP_LOGE(TAG, "ORP Point 1 calibration failed: could not get stable reading");
    }
}

void APAPHX_ADS1015::calibrate_orp_point2(float mv_value) {
    float voltage = get_stable_reading(orp_address_, orp_gain_);
    if (!std::isnan(voltage)) {
        cal_data_.orp_ref2_mv = voltage * 1000.0f;
        cal_data_.orp_ref2_value = mv_value;
        cal_data_.orp_last_calibration = ::time(nullptr);
        save_calibration_();
        update_calibration_age_();
        status_flags_.calibration_valid = true;
        ESP_LOGI(TAG, "ORP Point 2 calibrated: %.1f mV at %.1f mV", mv_value, voltage * 1000.0f);
    } else {
        ESP_LOGE(TAG, "ORP Point 2 calibration failed: could not get stable reading");
    }
}

void APAPHX_ADS1015::reset_calibration() {
    cal_data_ = {0, 0, 4.0f, 7.0f, 0, 0, 0, 475.0f, 650.0f, 0};
    save_calibration_();
    update_calibration_age_();
    status_flags_.calibration_valid = false;
    ESP_LOGI(TAG, "Calibration reset to defaults");
}

}  // namespace apaphx_ads1015
}  // namespace esphome