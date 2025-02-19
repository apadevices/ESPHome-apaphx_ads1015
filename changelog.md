# Changelog

All notable changes to the ESPHome APA PHX ADS1015 component will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-02-18
### Initial Release
First stable release of the ESPHome component for PHX-board with complete feature set:

### Core Features
- pH and ORP measurement using ADS1015 ADC
- Advanced signal processing with 100-sample averaging
- Two-point calibration system for both pH and ORP
- Non-volatile calibration storage
- Temperature compensation using two-point calibration
- Real-time monitoring and status reporting

### Monitoring Features
- Comprehensive pump state monitoring:
  - Direct GPIO pin monitoring
  - PHX_pumpstate device support
  - Flow sensor compatibility
  - External timer integration
- Filter pressure monitoring with thresholds:
  - Multiple pressure warning levels
  - Automatic backwash recommendations
  - Pressure trend tracking
- Calibration age tracking and warnings
- Status reporting system:
  - Water quality monitoring (pH/ORP)
  - Filter maintenance alerts
  - System health indicators

### Technical Implementation
- Non-blocking operation via state machine
- Configurable sampling system:
  - Adjustable sample count
  - Variable sampling intervals
  - Customizable stability thresholds
- Measurement validation:
  - pH range: 0-14
  - ORP range: 0-1000mV
  - Automatic error detection
- Robust I2C communication
- Native ESPHome/Home Assistant integration

### Documentation
- Comprehensive installation guide
- Detailed configuration examples:
  - Basic setup
  - Advanced features
  - Complete system integration
- Technical specifications
- Troubleshooting guide
- Maintenance recommendations