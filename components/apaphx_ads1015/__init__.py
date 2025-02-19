"""APA PHX ADS1015 component for pH and ORP measurements with temperature compensation."""
import esphome.codegen as cg
import esphome.automation as automation

DEPENDENCIES = ['i2c']
AUTO_LOAD = ['sensor']

# Service call identifiers
CONF_CALIBRATE_PH1 = "calibrate_ph1"
CONF_CALIBRATE_PH2 = "calibrate_ph2"
CONF_CALIBRATE_ORP1 = "calibrate_orp1"
CONF_CALIBRATE_ORP2 = "calibrate_orp2"
CONF_RESET_CALIBRATION = "reset_calibration"
CONF_VALUE = "value"

# Configuration options for smoothing and temperature
CONF_EMA_ALPHA = "ema_alpha"
CONF_MEDIAN_WINDOW = "median_window"
CONF_TEMPERATURE_SENSOR = "temperature_sensor"
CONF_PUMP_SENSOR = "pump_sensor"
CONF_ORP_VOLTAGE = "voltage_sensor"  # Added for ORP voltage sensor

# Configuration options for calibration age
CONF_PH_CAL_AGE = "ph_calibration_age"
CONF_ORP_CAL_AGE = "orp_calibration_age"

# Create namespace
apaphx_ads1015_ns = cg.esphome_ns.namespace('apaphx_ads1015')

# Actions for services
CalibratePh1Action = apaphx_ads1015_ns.class_('CalibratePh1Action', automation.Action)
CalibratePh2Action = apaphx_ads1015_ns.class_('CalibratePh2Action', automation.Action)
CalibrateOrp1Action = apaphx_ads1015_ns.class_('CalibrateOrp1Action', automation.Action)
CalibrateOrp2Action = apaphx_ads1015_ns.class_('CalibrateOrp2Action', automation.Action)
ResetCalibrationAction = apaphx_ads1015_ns.class_('ResetCalibrationAction', automation.Action)