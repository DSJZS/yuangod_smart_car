#include "./include/bdc_motor_control.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "low_pass_filter.h"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

static const char* TAG = "from -> "__FILE__;

/**
  * @brief 直流电机控制初始化(PWM配置， 4x正交解码器配置, PID配置)
  *
  * @param control_config 输入变量, 用于配置如何输出
  * @param control_context 输出变量, 控制上下文, 用于获取所需的句柄
  */
void dc_motor_control_init( dc_motor_control_config_t* control_config, dc_motor_control_context_t* control_context)
{
    ESP_LOGD( TAG, "初始化有刷直流电机中!!!");

    bdc_motor_config_t motor_config = {
        .pwma_gpio_num = control_config->pwm_gpio_a_num,
        .pwmb_gpio_num = control_config->pwm_gpio_b_num,
        .pwm_freq_hz = control_config->pwm_freq_hz,
    };
    control_context->compare_max = BDC_MCPWM_TIMER_RESOLUTION_HZ / control_config->pwm_freq_hz;
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    control_context->motor = motor;
    ESP_LOGD( TAG, "初始化有刷直流电机完毕!!!");

    ESP_LOGD( TAG, "初始化电机4倍频正交解码器中!!!");
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation，超出limit时软件累计以弥补误差
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = control_config->encoder_gpio_a_num,
        .level_gpio_num = control_config->encoder_gpio_b_num,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = control_config->encoder_gpio_b_num,
        .level_gpio_num = control_config->encoder_gpio_a_num,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    control_context->pcnt_encoder = pcnt_unit;
    ESP_LOGD( TAG, "初始化电机4倍频正交解码器中!!!");

    ESP_LOGD( TAG, "初始化电机PID控制块中!!!");

    float max_output_abs = (float)( (BDC_MCPWM_TIMER_RESOLUTION_HZ / control_config->pwm_freq_hz) - 1 );

    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = control_config->pid.kp,
        .ki = control_config->pid.ki,
        .kd = control_config->pid.kd,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   =  max_output_abs,
        .min_output   = -max_output_abs,
        .max_integral = control_config->pid.max_integral,
        .min_integral = control_config->pid.min_integral,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    control_context->pid_ctrl = pid_ctrl;
    control_context->last_pulses = 0;
    control_context->report_pulses = 0;
    control_context->target_pulses = 0;
    control_context->mutex = xSemaphoreCreateRecursiveMutex();
    ESP_LOGD( TAG, "初始化电机PID控制块完毕!!!");

    // ESP_LOGD( TAG, "初始化电机高分辨率定时器中!!!");
    // const esp_timer_create_args_t periodic_timer_args = {
    //     .callback = pid_loop_cb,
    //     .arg = control_context,
    //     .name = "pid_loop"
    // };
    // esp_timer_handle_t pid_loop_timer = NULL;
    // ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));
    // ESP_LOGD( TAG, "初始化电机高分辨率定时器完毕!!!");

    ESP_LOGD(TAG, "使能当前设置的电机");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGD(TAG, "将当前电机方向初始化为前进并设置默认速度为0");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));
    ESP_ERROR_CHECK(bdc_motor_set_speed( motor, 0));
    ESP_LOGD(TAG, "开启当前电机的速度控制高分辨率周期控制(通过task)");
    // ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, control_config->pid.period_ms * 1000));

    ESP_LOGI(TAG, "电机控制初始化完毕");
}

/**
  * @brief 设置目标脉冲值(线程安全)
  *
  * @param control_context 控制上下文
  * @param target_pulses 目标脉冲值
  */
void dc_motor_control_set_target( dc_motor_control_context_t* control_context, int target_pulses)
{
    xSemaphoreTakeRecursive( control_context->mutex, portMAX_DELAY);
    control_context->target_pulses = target_pulses;
    xSemaphoreGiveRecursive( control_context->mutex);
}

/**
  * @brief 更新脉冲值记录
  *
  * @param control_context 控制上下文
  * @return
  *      - real_pulses: 最新的脉冲值 
  */
int dc_motor_control_get_pulse_cnt( dc_motor_control_context_t* control_context)
{
    xSemaphoreTakeRecursive( control_context->mutex, portMAX_DELAY);

    pcnt_unit_handle_t pcnt_unit = control_context->pcnt_encoder;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - control_context->last_pulses;
    control_context->last_pulses = cur_pulse_count;


    /* 更新本论速度值(低通滤波) */
    control_context->report_pulses = (int)low_pass_filter( 0.3f, (float)control_context->report_pulses, (float)real_pulses);
    // control_context->report_pulses = real_pulses;

    xSemaphoreGiveRecursive( control_context->mutex);
    return control_context->report_pulses;
}

 /**
   * @brief 更新电机的速度(带方向)
   *
   * @param control_context 控制上下文
   * @param new_speed 新速度，分正负，正为正向，负为反向
   */
void bdc_motor_set_speed_with_direction( dc_motor_control_context_t* control_context, float new_speed)
{
    xSemaphoreTakeRecursive( control_context->mutex, portMAX_DELAY);
    if ( new_speed > 0 ) {
        bdc_motor_forward( control_context->motor);
    } else if ( new_speed < 0 ) {
        new_speed = -new_speed;
        bdc_motor_reverse( control_context->motor);
    } else {
        bdc_motor_brake( control_context->motor);
    }

    /* 驱动底层是直接更改比较值来改变速度的，此处要限制速度大小，防止超出范围(不然驱动底层会报错并阻止set_speed操作) */
    if( new_speed > control_context->compare_max )
        new_speed = control_context->compare_max;

    bdc_motor_set_speed( control_context->motor, (uint32_t)new_speed);
    xSemaphoreGiveRecursive( control_context->mutex);
}

/**
  * @brief 电机PID计算
  *
  * @param control_context 控制上下文
  * @return
  *      - new_speed: 计算而得的新速度
  */
float dc_motor_control_pid_compute( dc_motor_control_context_t* control_context)
{
    xSemaphoreTakeRecursive( control_context->mutex, portMAX_DELAY);
    float error = control_context->target_pulses - control_context->report_pulses;
    float new_speed = 0;

    // ESP_LOGD(TAG, "error: %f", error);

    pid_compute( control_context->pid_ctrl, error, &new_speed);

    // ESP_LOGD(TAG, "new_speed: %f", new_speed);
    
    // ESP_LOGD(TAG, "error:%d - %d = %f -> %f", control_context->target_pulses, control_context->report_pulses, error, new_speed);
    
    xSemaphoreGiveRecursive( control_context->mutex);
    
    return new_speed;
}