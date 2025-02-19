import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_ADDRESS,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_VOLTAGE,
    UNIT_VOLT,
)
from esphome.core import CORE
from esphome.automation import register_action
from . import (
    apaphx_ads1015_ns,
    CONF_CALIBRATE_PH1,
    CONF_CALIBRATE_PH2,
    CONF_CALIBRATE_ORP1,
    CONF_CALIBRATE_ORP2,
    CONF_RESET_CALIBRATION,
    CONF_VALUE,
    CONF_PH_CAL_AGE,
    CONF_ORP_CAL_AGE,
    CONF_EMA_ALPHA,
    CONF_MEDIAN_WINDOW,
    CONF_TEMPERATURE_SENSOR,
    CONF_PUMP_SENSOR,
    CalibratePh1Action, CalibratePh2Action,
    CalibrateOrp1Action, CalibrateOrp2Action,
    ResetCalibrationAction,
)

DEPENDENCIES = ['i2c']

CONF_GAIN = "gain"
CONF_ORP = "orp"
CONF_PH_SENSOR = "ph_sensor"
CONF_ORP_SENSOR = "orp_sensor"
CONF_ORP_VOLTAGE = "voltage_sensor"  # Added for ORP voltage sensor

UNIT_PH = "pH"
UNIT_MILLIVOLT = "mV"
UNIT_DAYS = "days"

APAPHX_ADS1015 = apaphx_ads1015_ns.class_('APAPHX_ADS1015', sensor.Sensor, cg.Component, i2c.I2CDevice)

GAIN_ENUM = {
    6: 0x0000,  # 6.144V
    4: 0x0200,  # 4.096V
    2: 0x0400,  # 2.048V
    1: 0x0600,  # 1.024V
    0: 0x0800,  # 0.512V
    -1: 0x0A00, # 0.256V
}

ORP_SCHEMA = cv.Schema({
    cv.Required(CONF_ADDRESS): cv.i2c_address,
    cv.Optional(CONF_GAIN, default=2): cv.enum(GAIN_ENUM),
    cv.Optional(CONF_ORP_SENSOR): sensor.sensor_schema(
        unit_of_measurement=UNIT_MILLIVOLT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT
    ),
    cv.Optional(CONF_ORP_VOLTAGE): sensor.sensor_schema(  # Added ORP voltage sensor schema
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT
    ),
})

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend({
        cv.GenerateID(): cv.declare_id(APAPHX_ADS1015),
        cv.Required(CONF_ADDRESS): cv.i2c_address,
        cv.Optional(CONF_GAIN, default=6): cv.enum(GAIN_ENUM),
        cv.Optional(CONF_EMA_ALPHA, default=0.1): cv.float_range(min=0.0, max=1.0),
        cv.Optional(CONF_MEDIAN_WINDOW, default=5): cv.int_range(min=3, max=20),
        cv.Optional(CONF_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_PUMP_SENSOR): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_PH_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_PH,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_ORP): ORP_SCHEMA,
        cv.Optional(CONF_PH_CAL_AGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DAYS,
            icon="mdi:clock-outline",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_ORP_CAL_AGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DAYS,
            icon="mdi:clock-outline",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT
        ),
    })
    .extend(cv.polling_component_schema("5s"))
    .extend(i2c.i2c_device_schema(0x48))
)

CALIBRATION_ACTION_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.use_id(APAPHX_ADS1015),
    cv.Required(CONF_VALUE): cv.float_,
})

RESET_ACTION_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.use_id(APAPHX_ADS1015),
})

@register_action(CONF_CALIBRATE_PH1, CalibratePh1Action, CALIBRATION_ACTION_SCHEMA)
async def calibrate_ph1_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren, config[CONF_VALUE])

@register_action(CONF_CALIBRATE_PH2, CalibratePh2Action, CALIBRATION_ACTION_SCHEMA)
async def calibrate_ph2_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren, config[CONF_VALUE])

@register_action(CONF_CALIBRATE_ORP1, CalibrateOrp1Action, CALIBRATION_ACTION_SCHEMA)
async def calibrate_orp1_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren, config[CONF_VALUE])

@register_action(CONF_CALIBRATE_ORP2, CalibrateOrp2Action, CALIBRATION_ACTION_SCHEMA)
async def calibrate_orp2_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren, config[CONF_VALUE])

@register_action(CONF_RESET_CALIBRATION, ResetCalibrationAction, RESET_ACTION_SCHEMA)
async def reset_calibration_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
    await i2c.register_i2c_device(var, config)
    
    if CONF_TEMPERATURE_SENSOR in config:
        temp_sens = await cg.get_variable(config[CONF_TEMPERATURE_SENSOR])
        cg.add(var.set_temperature_sensor(temp_sens))

    if CONF_PUMP_SENSOR in config:
        pump_sens = await cg.get_variable(config[CONF_PUMP_SENSOR])
        cg.add(var.set_pump_sensor(pump_sens))
    
    cg.add(var.set_ph_address(config[CONF_ADDRESS]))
    cg.add(var.set_ph_gain(config[CONF_GAIN]))
    cg.add(var.set_ema_alpha(config[CONF_EMA_ALPHA]))
    cg.add(var.set_median_window(config[CONF_MEDIAN_WINDOW]))
    
    if CONF_PH_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_PH_SENSOR])
        cg.add(var.set_ph_sensor(sens))
    
    if CONF_ORP in config:
        orp_config = config[CONF_ORP]
        if CONF_ORP_SENSOR in orp_config:
            orp_sens = await sensor.new_sensor(orp_config[CONF_ORP_SENSOR])
            cg.add(var.set_orp_sensor(orp_sens))
        if CONF_ORP_VOLTAGE in orp_config:  # Added handling for ORP voltage sensor
            orp_volt_sens = await sensor.new_sensor(orp_config[CONF_ORP_VOLTAGE])
            cg.add(var.set_orp_voltage_sensor(orp_volt_sens))
        cg.add(var.set_orp_address(orp_config[CONF_ADDRESS]))
        cg.add(var.set_orp_gain(orp_config[CONF_GAIN]))
        
    if CONF_PH_CAL_AGE in config:
        ph_age_sens = await sensor.new_sensor(config[CONF_PH_CAL_AGE])
        cg.add(var.set_ph_cal_age_sensor(ph_age_sens))
        
    if CONF_ORP_CAL_AGE in config:
        orp_age_sens = await sensor.new_sensor(config[CONF_ORP_CAL_AGE])
        cg.add(var.set_orp_cal_age_sensor(orp_age_sens))