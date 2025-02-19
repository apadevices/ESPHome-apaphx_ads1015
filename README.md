# ESPHome APA PHX ADS1015 Component

A comprehensive ESPHome component for swimming pool water quality monitoring using ADS1015 ADC. This component provides accurate pH and ORP (Redox) measurements with advanced features for reliable home pool management through Home Assistant.

## Key Features

### Primary Hardware Support
- **PHX-board by APA Devices**:
  - Optimized for PHX-board's circuit design and components
  - Pre-configured gain settings for optimal performance
  - Validated temperature compensation curves
  - Tested with PHX-board's sensor configurations
  - Simplified setup process for PHX-board users

### Universal Compatibility
As a secondary benefit, this component supports:
- Any analog pH and ORP sensors when paired with ADS1015 ADC
- Flexible configuration for different sensor types and ranges
- Independent dual-ADC support for simultaneous pH and ORP monitoring
- Custom gain settings for various sensor outputs

### Advanced Measurement Features
- High-precision readings through sophisticated signal processing:
  - Multi-sample averaging (100 samples per reading)
  - Configurable rolling average buffer
  - Automatic noise filtering and outlier rejection
  - Stability detection for reliable calibration
- Temperature compensation for pH measurements:
  - Works without direct temperature sensor dependency
  - Uses default pool temperature if no sensor available
  - Automatic compensation curves for common pH sensors

### Professional Calibration System
- Two-point calibration for both pH and ORP sensors
- Non-volatile storage of calibration data
- Calibration stability verification
- Easy recalibration procedure through Home Assistant
- Pre-configured defaults for common calibration solutions:
  - pH: 4.0 and 7.0 buffer solutions
  - ORP: 475mV and 650mV reference solutions

### Complete Pool Monitoring Solution
- Essential parameters for pool maintenance:
  - pH level (0-14 range with 0.01 resolution)
  - ORP/Redox potential (0-1000mV range with 1mV resolution)
  - Filter pressure monitoring capability
  - Integration ready for additional sensors

### Pump Monitoring & Dependencies
- Flexible pump state monitoring options:
  - Direct ESP pin connection for simple setups
  - PHX_pumpstate device support for AC voltage monitoring
  - Water flow sensor compatibility
  - External timer integration
- Smart measurement dependencies:
  - Configurable pump running validation before measurements
  - Prevents inaccurate readings during pump startup/shutdown
  - Customizable delay timers for measurement stability
  - Automatic measurement skip when pump is inactive
- Multiple monitoring methods:
  - Direct pump motor voltage detection
  - Flow sensor-based validation
  - External control signal monitoring
  - Timer-based scheduling integration
- Benefits:
  - Prevents invalid measurements during water stagnation
  - Ensures accurate readings during proper water circulation
  - Reduces sensor wear during pump-off periods
  - Supports various pool system configurations

### ESPHome Integration
- Native Home Assistant integration
- Real-time sensor updates
- Automatic sensor discovery
- Built-in diagnostic features
- Comprehensive error reporting

## Installation

1. Copy component files to your ESPHome config directory:
   ```
   config/
   └── esphome/
       └── components/
           └── apaphx_ads1015/
               ├── __init__.py
               └── apaphx_ads1015.h
   ```

2. Add configuration to your ESPHome YAML:
   ```yaml
   # Example configuration for PHX-board
   external_components:
     - source: components
       components: [apaphx_ads1015]
   
   apaphx_ads1015:
     ph:
       name: "Pool pH"
     orp:
       name: "Pool ORP"
     # Optional configurations (pre-configured for PHX-board)
     ph_address: 0x48  # Default I2C address for PHX pH ADC
     orp_address: 0x49 # Default I2C address for PHX ORP ADC
     ph_gain: 1        # Optimized for PHX-board pH sensor
     orp_gain: 1       # Optimized for PHX-board ORP sensor
   ```

## Usage

### Initial Setup
1. For PHX-board:
   - Connect your PHX-board to power and I2C
   - Use default configuration values
   - Upload the configuration to your ESP device
   
   For other hardware:
   - Wire your pH and ORP sensors to separate ADS1015 ADCs
   - Configure appropriate I2C addresses and gains
   - Validate sensor output ranges

2. The sensors will appear automatically in Home Assistant

### Calibration Process
1. In Home Assistant, navigate to the device page
2. Select "Calibrate pH" or "Calibrate ORP" service
3. Follow the calibration wizard:
   - For pH: Use pH 4.0 and pH 7.0 buffer solutions
   - For ORP: Use 475mV and 650mV reference solutions
4. Wait for stability confirmation
5. Save calibration

### Maintenance Recommendations
- Calibrate pH sensor monthly
- Calibrate ORP sensor quarterly
- Clean sensors according to manufacturer guidelines
- Monitor filter pressure trends for maintenance scheduling
- Keep calibration solutions fresh and properly stored

## Technical Details

### Signal Processing
- 100 samples per reading with 10ms intervals
- Rolling average buffer (configurable size)
- Stability threshold: 0.5 units
- Automatic gain selection for optimal resolution
- Non-blocking operation through state machine design

### Measurement Specifications
- pH Range: 0-14 pH
- pH Resolution: 0.01 pH
- pH Accuracy: ±0.1 pH (after calibration)
- ORP Range: 0-1000 mV
- ORP Resolution: 1 mV
- ORP Accuracy: ±5 mV (after calibration)

### Temperature Compensation
The component implements two-point temperature compensation for pH measurements:
- Uses calibration curve based on two temperature points
- More accurate than linear compensation across temperature range
- Compensates for both sensor and solution temperature effects
- Optional external temperature sensor input
- Default reference temperature: 25°C
- Supported temperature range: 5°C to 40°C

## Troubleshooting

### Common Issues
1. Unstable Readings
   - Check sensor connections
   - Verify ADC gain settings
   - Ensure proper grounding
   - Clean sensor probes

2. Calibration Failures
   - Use fresh calibration solutions
   - Clean sensor before calibration
   - Allow sufficient stabilization time
   - Check temperature stability

3. Communication Errors
   - Verify I2C addresses
   - Check wiring connections
   - Validate power supply stability

### Diagnostic Tools
- Built-in voltage monitoring
- Stability indicators
- Error logging
- Raw ADC value access

## Contributing
Contributions are welcome! Please submit pull requests with:
- Bug fixes
- Feature improvements
- Documentation updates
- Additional sensor support

## License
This component is released under the MIT License.

## Acknowledgments
- APA Devices for PHX-board hardware design
- ESPHome community for testing and feedback
- ADS1015 driver contributors
- Home Assistant community for integration testing