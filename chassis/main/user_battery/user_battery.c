#include "user_battery.h"
#include "adc_battery_estimation.h"
#include "driver/adc.h"

#define BAT_VOLTAGE_ADC_CHANNEL ADC1_CHANNEL_7
#define BAT_CURRENT_ADC_CHANNEL ADC1_CHANNEL_8

static adc_battery_estimation_handle_t adc_battery_estimation_handle;

void battery_estimation_init(void)
{
    adc_battery_estimation_t config = {
        .internal = {
            .adc_unit = ADC_UNIT_1,
            .adc_bitwidth = ADC_BITWIDTH_DEFAULT,
            .adc_atten = ADC_ATTEN_DB_6,
        },
        .adc_channel = BAT_VOLTAGE_ADC_CHANNEL,
        .lower_resistor = 100 * 1000,
        .upper_resistor = 100 * 1000,
    };

    adc_battery_estimation_handle = adc_battery_estimation_create(&config);
}

void battery_get_capacity(float *capacity)
{
    adc_battery_estimation_get_capacity( adc_battery_estimation_handle, capacity);
}

