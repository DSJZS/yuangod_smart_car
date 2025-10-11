#include "user_motors.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_priorities.h"
#include "task_period.h"

static const char* TAG = "from -> "__FILE__;

/* 主动轮轮距，单位：m */
#define WHEEL_SEPARATION    ( 0.073f )      //  73mm
/* 主动轮周长，单位: m */
#define WHEEL_PERIMETER     ( 0.13195f )    //  直径 42mm
/* 主动轮每转脉冲 */
#define WHEEL_PER_ROUND     ( 50.0f * 7.0f * 4.0f ) //  减速比 50、 基础脉冲数(分辨率) 7ppr、 四倍频

dc_motor_control_context_t left_motor_dev;
dc_motor_control_context_t right_motor_dev;
static QueueHandle_t motor_mailbox = NULL;

typedef struct {
    int left_pulses;
    int right_pulses;
}motor_data_t;



static float motor_pulses2mps(int pulses)
{
    float mps = (float)pulses;
    mps /= ( WHEEL_PER_ROUND ); //  减速比 50、 基础脉冲数(分辨率) 7ppr、 四倍频, 此布得圈数
    mps *= WHEEL_PERIMETER; // 轮子直径42mm 求得周长，此步得位移
    mps /= ( MOTOR_TASK_PERIOD / 1000.0f ); //  时间单位为 MOTOR_TASK_PERIOD ，此步得速度
    return mps;
}

static int motor_mps2pulses(float mps)
{
    float pulses = mps;
    pulses *= ( MOTOR_TASK_PERIOD / 1000.0f );  //  //  时间单位为 MOTOR_TASK_PERIOD, 得到每时间单位位移
    pulses /= WHEEL_PERIMETER; // 轮子直径42mm 求得周长，此步得圈数
    pulses *= ( WHEEL_PER_ROUND ); //  减速比 50、 基础脉冲数(分辨率) 7ppr、 四倍频, 此布得脉冲
    return (int)pulses;
}

static void motor_task(void* param)
{
    const TickType_t xDelay = pdMS_TO_TICKS( MOTOR_TASK_PERIOD );
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        int left_pulses = 0, right_pulses = 0;
        
        float new_speed = 0;

        left_pulses = dc_motor_control_get_pulse_cnt( &left_motor_dev);
        ESP_LOGI(TAG, "left cnt = %d", left_pulses);
        right_pulses = dc_motor_control_get_pulse_cnt( &right_motor_dev);
        ESP_LOGI(TAG, "right cnt = %d", right_pulses);

        motor_data_t motor_data = {
            .left_pulses = left_pulses,
            .right_pulses = right_pulses,
        };
        xQueueOverwrite( motor_mailbox, &motor_data);

        // new_speed = dc_motor_control_pid_compute( &left_motor_dev);
        bdc_motor_set_speed_with_direction( &left_motor_dev, new_speed);

        // new_speed = dc_motor_control_pid_compute( &right_motor_dev);
        bdc_motor_set_speed_with_direction( &right_motor_dev, new_speed);
        
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
            .kp = 40.0f,
            .ki = 0.0f,
            .kd = 100.0f,
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

    motor_mailbox = xQueueCreate( 1, sizeof(motor_data_t) );
    xTaskCreatePinnedToCore( motor_task, "motor_task", 4096, NULL, MOTOR_TASK_PRIO, NULL, 1);
}


//  m/s rad/s 右手螺旋
void motor_set_speed( float linear_x, float angular_z)
{
    int left_target = 0, right_target = 0;
    float left_speed = 0, right_speed = 0;

    left_speed = linear_x - angular_z * WHEEL_SEPARATION / 2.0f;
    right_speed = linear_x + angular_z * WHEEL_SEPARATION / 2.0f;

    left_target = motor_mps2pulses(left_speed);
    right_target = motor_mps2pulses(right_speed);

    dc_motor_control_set_target( &left_motor_dev, left_target);
    dc_motor_control_set_target( &right_motor_dev, right_target);
}

//  m/s rad/s 右手螺旋
void motor_get_speed( float* linear_x, float* angular_z)
{
    motor_data_t motor_data = {0};
    float left_speed = 0, right_speed = 0;
    
    xQueuePeek( motor_mailbox, &motor_data, portMAX_DELAY);

    left_speed = motor_pulses2mps( motor_data.left_pulses );
    right_speed = motor_pulses2mps( motor_data.right_pulses );

    *linear_x = ( left_speed + right_speed ) / 2.0f;
    *angular_z = (right_speed - left_speed) / WHEEL_SEPARATION;
}