#include "user_motors.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_priorities.h"
#include "task_period.h"

static const char* TAG = "from -> "__FILE__;

dc_motor_control_context_t left_motor_dev;
dc_motor_control_context_t right_motor_dev;

void motor_task(void* param)
{
    const TickType_t xDelay = pdMS_TO_TICKS( MOTOR_TASK_PERIOD );
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        dc_motor_control_get_pulse_cnt( &left_motor_dev);
        // ESP_LOGI(TAG, "left cnt = %d", left_motor_dev.report_pulses);
        dc_motor_control_get_pulse_cnt( &right_motor_dev);
        // ESP_LOGI(TAG, "right cnt = %d", right_motor_dev.report_pulses);
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }

    vTaskDelete(NULL);
}

void motor_init(void)
{
    dc_motor_control_config_t left_motor_config = {
        .pwm_gpio_a_num = GPIO_NUM_33,
        .pwm_gpio_b_num = GPIO_NUM_34,
        .pwm_freq_hz = 25000,
        .encoder_gpio_a_num = GPIO_NUM_12,
        .encoder_gpio_b_num = GPIO_NUM_14,
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
        .pwm_gpio_a_num = GPIO_NUM_35,
        .pwm_gpio_b_num = GPIO_NUM_36,
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
    xTaskCreatePinnedToCore( motor_task, "motor_task", 4096, NULL, MOTOR_TASK_PRIO, NULL, 1);
}

void motor_set_target( dc_motor_control_context_t* control_context, int target_pulses)
{
    dc_motor_control_set_target( control_context, target_pulses);
}
