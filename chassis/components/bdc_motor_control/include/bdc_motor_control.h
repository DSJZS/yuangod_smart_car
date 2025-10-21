#ifndef __YG_DC_MOTOR_CONTROL_H__
#define __YG_DC_MOTOR_CONTROL_H__

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "driver/pulse_cnt.h"

/* 用于DRV8833与DRV8848等系列，不适用于TB6612等等 */

typedef struct {
    float kp;                      // PID Kp parameter
    float ki;                      // PID Ki parameter
    float kd;                      // PID Kd parameter
    float max_integral;            // PID maximum integral value limitation
    float min_integral;            // PID minimum integral value limitation
    uint64_t period_ms;            // PID计算周期(单位：ms)
}pid_config_t;

typedef struct {
    uint32_t pwm_gpio_a_num;    //  Pwm A通道的GPIO号
    uint32_t pwm_gpio_b_num;    //  Pwm B通道的GPIO号
    uint32_t pwm_freq_hz;       //  PWM频率
    int encoder_gpio_a_num;     //  Encoder A通道的GPIO号
    int encoder_gpio_b_num;     //  Encoder B通道的GPIO号
    pid_config_t pid;           //  pid参数设置
} dc_motor_control_config_t;

typedef struct {
    bdc_motor_handle_t motor;           //  电机驱动句柄
    pcnt_unit_handle_t pcnt_encoder;    //  脉冲计数器句柄
    pid_ctrl_block_handle_t pid_ctrl;   //  pid控制块句柄
    int report_pulses;                  //  本轮脉冲计数
    int last_pulses;                    //  上轮脉冲计数
    int target_pulses;                  //  目标脉冲计数
    uint32_t compare_max;               //  比较值的最大值
    SemaphoreHandle_t mutex;            //  互斥量(用于确保线程安全)
} dc_motor_control_context_t;

/* 初始化函数 */
void dc_motor_control_init( dc_motor_control_config_t* control_config, dc_motor_control_context_t* control_context);
/* 修改目标脉冲值(线程安全) */
void dc_motor_control_set_target( dc_motor_control_context_t* control_context, int target_pulses);
/* 更新脉冲值记录(线程安全) */
int dc_motor_control_get_pulse_cnt( dc_motor_control_context_t* control_context);
/* 电机PID计算 */
float dc_motor_control_pid_compute( dc_motor_control_context_t* control_context);
/* 电机直接控制 */
void bdc_motor_set_speed_with_direction( dc_motor_control_context_t* control_context, float new_speed);


#endif
