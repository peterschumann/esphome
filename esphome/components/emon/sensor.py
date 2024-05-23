import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.core import CORE, TimePeriod
from esphome.components import sensor
from esphome.components.esp32 import get_esp32_variant
from esphome.const import (
    CONF_ATTENUATION,
    CONF_ID,
    CONF_NUMBER,
    CONF_PIN,
    CONF_WIFI,
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_ACTIVE_POWER,
    CONF_APPARENT_POWER,
    CONF_POWER_FACTOR,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_APPARENT_POWER,
    DEVICE_CLASS_POWER_FACTOR,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_VOLT_AMPS,
)
from esphome.components.adc import (
    ATTENUATION_MODES,
    validate_adc_pin,
)
from esphome.components.esp32.const import (
    VARIANT_ESP32,
    VARIANT_ESP32C2,
    VARIANT_ESP32C3,
    VARIANT_ESP32C6,
    VARIANT_ESP32H2,
    VARIANT_ESP32S2,
    VARIANT_ESP32S3,
)

adc_unit_t = cg.global_ns.enum("adc_unit_t")

adc_channel_t = cg.global_ns.enum("adc_channel_t")

# From https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/adc_common.h
# pin to adc1 channel mapping
ESP32_VARIANT_ADC1_PIN_TO_CHANNEL = {
    VARIANT_ESP32: {
        36: adc_channel_t.ADC_CHANNEL_0,
        37: adc_channel_t.ADC_CHANNEL_1,
        38: adc_channel_t.ADC_CHANNEL_2,
        39: adc_channel_t.ADC_CHANNEL_3,
        32: adc_channel_t.ADC_CHANNEL_4,
        33: adc_channel_t.ADC_CHANNEL_5,
        34: adc_channel_t.ADC_CHANNEL_6,
        35: adc_channel_t.ADC_CHANNEL_7,
    },
    VARIANT_ESP32S2: {
        1: adc_channel_t.ADC_CHANNEL_0,
        2: adc_channel_t.ADC_CHANNEL_1,
        3: adc_channel_t.ADC_CHANNEL_2,
        4: adc_channel_t.ADC_CHANNEL_3,
        5: adc_channel_t.ADC_CHANNEL_4,
        6: adc_channel_t.ADC_CHANNEL_5,
        7: adc_channel_t.ADC_CHANNEL_6,
        8: adc_channel_t.ADC_CHANNEL_7,
        9: adc_channel_t.ADC_CHANNEL_8,
        10: adc_channel_t.ADC_CHANNEL_9,
    },
    VARIANT_ESP32S3: {
        1: adc_channel_t.ADC_CHANNEL_0,
        2: adc_channel_t.ADC_CHANNEL_1,
        3: adc_channel_t.ADC_CHANNEL_2,
        4: adc_channel_t.ADC_CHANNEL_3,
        5: adc_channel_t.ADC_CHANNEL_4,
        6: adc_channel_t.ADC_CHANNEL_5,
        7: adc_channel_t.ADC_CHANNEL_6,
        8: adc_channel_t.ADC_CHANNEL_7,
        9: adc_channel_t.ADC_CHANNEL_8,
        10: adc_channel_t.ADC_CHANNEL_9,
    },
    VARIANT_ESP32C3: {
        0: adc_channel_t.ADC_CHANNEL_0,
        1: adc_channel_t.ADC_CHANNEL_1,
        2: adc_channel_t.ADC_CHANNEL_2,
        3: adc_channel_t.ADC_CHANNEL_3,
        4: adc_channel_t.ADC_CHANNEL_4,
    },
    VARIANT_ESP32C2: {
        0: adc_channel_t.ADC_CHANNEL_0,
        1: adc_channel_t.ADC_CHANNEL_1,
        2: adc_channel_t.ADC_CHANNEL_2,
        3: adc_channel_t.ADC_CHANNEL_3,
        4: adc_channel_t.ADC_CHANNEL_4,
    },
    VARIANT_ESP32C6: {
        0: adc_channel_t.ADC_CHANNEL_0,
        1: adc_channel_t.ADC_CHANNEL_1,
        2: adc_channel_t.ADC_CHANNEL_2,
        3: adc_channel_t.ADC_CHANNEL_3,
        4: adc_channel_t.ADC_CHANNEL_4,
        5: adc_channel_t.ADC_CHANNEL_5,
        6: adc_channel_t.ADC_CHANNEL_6,
    },
    VARIANT_ESP32H2: {
        0: adc_channel_t.ADC_CHANNEL_0,
        1: adc_channel_t.ADC_CHANNEL_1,
        2: adc_channel_t.ADC_CHANNEL_2,
        3: adc_channel_t.ADC_CHANNEL_3,
        4: adc_channel_t.ADC_CHANNEL_4,
    },
}

ESP32_VARIANT_ADC2_PIN_TO_CHANNEL = {
    # TODO: add other variants
    VARIANT_ESP32: {
        4: adc_channel_t.ADC_CHANNEL_0,
        0: adc_channel_t.ADC_CHANNEL_1,
        2: adc_channel_t.ADC_CHANNEL_2,
        15: adc_channel_t.ADC_CHANNEL_3,
        13: adc_channel_t.ADC_CHANNEL_4,
        12: adc_channel_t.ADC_CHANNEL_5,
        14: adc_channel_t.ADC_CHANNEL_6,
        27: adc_channel_t.ADC_CHANNEL_7,
        25: adc_channel_t.ADC_CHANNEL_8,
        26: adc_channel_t.ADC_CHANNEL_9,
    },
    VARIANT_ESP32S2: {
        11: adc_channel_t.ADC_CHANNEL_0,
        12: adc_channel_t.ADC_CHANNEL_1,
        13: adc_channel_t.ADC_CHANNEL_2,
        14: adc_channel_t.ADC_CHANNEL_3,
        15: adc_channel_t.ADC_CHANNEL_4,
        16: adc_channel_t.ADC_CHANNEL_5,
        17: adc_channel_t.ADC_CHANNEL_6,
        18: adc_channel_t.ADC_CHANNEL_7,
        19: adc_channel_t.ADC_CHANNEL_8,
        20: adc_channel_t.ADC_CHANNEL_9,
    },
    VARIANT_ESP32S3: {
        11: adc_channel_t.ADC_CHANNEL_0,
        12: adc_channel_t.ADC_CHANNEL_1,
        13: adc_channel_t.ADC_CHANNEL_2,
        14: adc_channel_t.ADC_CHANNEL_3,
        15: adc_channel_t.ADC_CHANNEL_4,
        16: adc_channel_t.ADC_CHANNEL_5,
        17: adc_channel_t.ADC_CHANNEL_6,
        18: adc_channel_t.ADC_CHANNEL_7,
        19: adc_channel_t.ADC_CHANNEL_8,
        20: adc_channel_t.ADC_CHANNEL_9,
    },
    VARIANT_ESP32C3: {
        5: adc_channel_t.ADC_CHANNEL_0,
    },
}


def final_validate_config(config):
    if not CORE.is_esp32:
        raise cv.Invalid("Only ESP32 is supported at the moment")

    variant = get_esp32_variant()
    if CONF_WIFI in fv.full_config.get() and any(
        [
            (pin := config[k][CONF_PIN][CONF_NUMBER])
            in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
            for k in [CONF_V_INPUT, CONF_I_INPUT]
        ]
    ):
        raise cv.Invalid(
            f"{variant} doesn't support ADC on pin {pin} when Wi-Fi is configured"
        )

    return config


emon_ns = cg.esphome_ns.namespace("emon")
Emon = emon_ns.class_("Emon", cg.Component)

CONF_CALIBRATION = "cal"

channel_schema = cv.Schema(
    {
        cv.Required(CONF_PIN): validate_adc_pin,
        cv.SplitDefault(CONF_ATTENUATION, esp32="0db"): cv.All(
            cv.only_on_esp32, cv.enum(ATTENUATION_MODES, lower=True)
        ),
        cv.Optional(CONF_CALIBRATION, default=1.0): cv.Coerce(float),
    },
)

CONF_V_INPUT = "v_input"
CONF_I_INPUT = "i_input"

CONF_DATA_INTERVAL = "data_interval"

CONFIG_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.declare_id(Emon),
        cv.Required(CONF_V_INPUT): channel_schema,
        cv.Required(CONF_I_INPUT): channel_schema,
        cv.Optional(
            CONF_DATA_INTERVAL, default=TimePeriod(milliseconds=200)
        ): cv.positive_not_null_time_period,
        cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ACTIVE_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_APPARENT_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_APPARENT_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER_FACTOR): sensor.sensor_schema(
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_POWER_FACTOR,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)

FINAL_VALIDATE_SCHEMA = final_validate_config


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add_define("DATA_INTERVAL_MS", config[CONF_DATA_INTERVAL].total_milliseconds)

    # if config[CONF_PIN] == "VCC":
    #     cg.add_define("USE_ADC_SENSOR_VCC")
    # elif config[CONF_PIN] == "TEMPERATURE":
    #     cg.add(var.set_is_temperature())
    # else:
    #     pin = await cg.gpio_pin_expression(config[CONF_PIN])
    #     cg.add(var.set_pin(pin))

    # cg.add(var.set_output_raw(config[CONF_RAW]))

    # if attenuation := config.get(CONF_ATTENUATION):
    #     if attenuation == "auto":
    #         cg.add(var.set_autorange(cg.global_ns.true))
    #     else:
    #         cg.add(var.set_attenuation(attenuation))

    if CORE.is_esp32:
        variant = get_esp32_variant()
        unit = None
        chan = None
        for dim, set_dim in [(CONF_V_INPUT, var.set_v), (CONF_I_INPUT, var.set_i)]:
            pin_num = config[dim][CONF_PIN][CONF_NUMBER]
            if (
                variant in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL
                and pin_num in ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant]
            ):
                unit = adc_unit_t.ADC_UNIT_1
                chan = ESP32_VARIANT_ADC1_PIN_TO_CHANNEL[variant][pin_num]
            elif (
                variant in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL
                and pin_num in ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant]
            ):
                unit = adc_unit_t.ADC_UNIT_2
                chan = ESP32_VARIANT_ADC2_PIN_TO_CHANNEL[variant][pin_num]

            if unit is not None and chan is not None:
                cg.add(
                    set_dim(
                        unit,
                        chan,
                        config[dim].get(CONF_ATTENUATION),
                        config[dim].get(CONF_CALIBRATION),
                    )
                )
        for dim in [
            CONF_VOLTAGE,
            CONF_CURRENT,
            CONF_ACTIVE_POWER,
            CONF_APPARENT_POWER,
            CONF_POWER_FACTOR,
        ]:
            if dim in config:
                sens = await sensor.new_sensor(config[dim])
                cg.add(getattr(var, f"set_{dim}_sensor")(sens))
