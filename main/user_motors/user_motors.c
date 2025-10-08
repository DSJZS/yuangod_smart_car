#include "user_motors.h"
#include "driver/gpio.h"

void motor_init(void)
{
    dc_motor_control_config_t left_motor_config = {
        .pwm_gpio_a_num = GPIO_NUM_33,
        .pwm_gpio_b_num = GPIO_NUM_34,
        .pwm_freq_hz = 25000,
        .encoder_gpio_a_num = GPIO_NUM_14,
        .encoder_gpio_b_num = GPIO_NUM_12,
        .pid = {
            .kp = 0.6,
            .ki = 0.4,
            .kd = 0.2,
            .max_integral = 1000,
            .min_integral = -1000,
            .period_ms = 5,
        }
    };
    dc_motor_control_config_t right_motor_config = {
        .pwm_gpio_a_num = GPIO_NUM_36,
        .pwm_gpio_b_num = GPIO_NUM_35,
        .pwm_freq_hz = 25000,
        .encoder_gpio_a_num = GPIO_NUM_20,
        .encoder_gpio_b_num = GPIO_NUM_19,
        .pid = {
            .kp = 0.6,
            .ki = 0.4,
            .kd = 0.2,
            .max_integral = 1000,
            .min_integral = -1000,
            .period_ms = 5,
        }
    };
    dc_motor_control_init( &left_motor_config, &left_motor_dev);
    dc_motor_control_init( &right_motor_config, &right_motor_dev);
}

void motor_set_target( dc_motor_control_context_t* control_context, int target_pulses)
{
    dc_motor_control_set_target( control_context, target_pulses);
}
