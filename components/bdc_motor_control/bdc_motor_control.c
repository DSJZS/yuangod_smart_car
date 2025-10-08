#include "bdc_motor_control.h"
#include "esp_log.h"
#include "esp_timer.h"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

static const char* TAG = "from -> "__FILE__;

static void bdc_motor_set_speed_with_direction( bdc_motor_handle_t motor, float new_speed)
{
    if ( new_speed > 0 ) {
        bdc_motor_forward( motor);
    } else if ( new_speed < 0 ) {
        new_speed = -new_speed;
        bdc_motor_reverse( motor);
    } else {
        bdc_motor_brake( motor);
    }

    bdc_motor_set_speed( motor, (uint32_t)new_speed);
}

/**
  * @brief pid控制回调(只能在任务中使用，不要在中断中使用， 线程安全)
  *
  * @param args dc_motor_control_context_t类型数据
  */
static void pid_loop_cb(void *args)
{
    dc_motor_control_context_t *ctx = (dc_motor_control_context_t *)args;
    xSemaphoreTake( ctx->mutex, portMAX_DELAY);

    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - ctx->last_pulses;
    ctx->last_pulses = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // calculate the speed error
    float error = ctx->target_pulses - real_pulses;
    float new_speed = 0;

    // ESP_LOGI(TAG, "error:%f", error);

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed_with_direction(motor, new_speed);

    xSemaphoreGive( ctx->mutex);
}

/**
  * @brief 直流电机控制初始化(PWM配置， 4x正交解码器配置, PID配置)
  *
  * @param control_config 输入变量, 用于配置如何输出
  * @param control_context 输出变量, 控制上下文, 用于获取所需的句柄
  */
void dc_motor_control_init( dc_motor_control_config_t* control_config, dc_motor_control_context_t* control_context)
{
    ESP_LOGI( TAG, "初始化有刷直流电机中!!!");

    bdc_motor_config_t motor_config = {
        .pwma_gpio_num = control_config->pwm_gpio_a_num,
        .pwmb_gpio_num = control_config->pwm_gpio_b_num,
        .pwm_freq_hz = control_config->pwm_freq_hz,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    control_context->motor = motor;
    ESP_LOGI( TAG, "初始化有刷直流电机完毕!!!");

    ESP_LOGI( TAG, "初始化电机正交解码器中!!!");
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
    ESP_LOGI( TAG, "初始化电机正交解码器完毕!!!");

    ESP_LOGI( TAG, "初始化电机PID控制块中!!!");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = control_config->pid.kp,
        .ki = control_config->pid.ki,
        .kd = control_config->pid.kd,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = (BDC_MCPWM_TIMER_RESOLUTION_HZ / control_config->pwm_freq_hz) - 1,
        .min_output   = 0,
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
    control_context->mutex = xSemaphoreCreateMutex();
    ESP_LOGI( TAG, "初始化电机PID控制块完毕!!!");

    ESP_LOGI( TAG, "初始化电机高分辨率定时器中!!!");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = control_context,
        .name = "pid_loop"
    };
    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));
    ESP_LOGI( TAG, "初始化电机高分辨率定时器完毕!!!");

    ESP_LOGI(TAG, "使能当前设置的电机");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "将当前电机方向初始化为前进并设置默认速度为0");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));
    ESP_ERROR_CHECK(bdc_motor_set_speed( motor, 0));
    ESP_LOGI(TAG, "开启当前电机的速度控制高分辨率周期控制(通过task)");
    // ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, control_config->pid.period_ms * 1000));
}

/**
  * @brief 设置目标脉冲值(线程安全)
  *
  * @param control_context 控制上下文
  * @param target_pulses 目标脉冲值
  */
void dc_motor_control_set_target( dc_motor_control_context_t* control_context, int target_pulses)
{
    xSemaphoreTake( control_context->mutex, portMAX_DELAY);
    control_context->target_pulses = target_pulses;
    xSemaphoreGive( control_context->mutex);
}
