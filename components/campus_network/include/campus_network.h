#ifndef __CAMPUS_NETWORK_H__
#define __CAMPUS_NETWORK_H__

#include "esp_event.h"
#include "freertos/FreeRTOS.h"

/* 校园网配置结构体 */
typedef struct {
    const char* can_ssid;                   //  校园网名称, 如 "CAMPUS_NET"
    // const char* can_password;            //  校园网密码, 校园网一般是开放认证, 故这里没考虑有密码的情况
    const char* can_user_id;                //  校园网用户id, 一般是你的学号或者证件号, 看学校安排
    const char* can_user_password;          //  校园网用户密码, 一般不为空故这里没有做空密码处理
    esp_event_handler_t can_event_handler;  //  用于处理wifi或者网络事件, 填入NULL表示用本函数库的回调处理
    UBaseType_t can_task_priority;          //  校园网后台任务(campus_network_task)的优先级,该任务用与处理连认证与重连

} campus_network_config_t;

/* 校园网连接初始化函数
 * 函数内部不会执行 nvs_flash_init() 等 nvs 操作
 * 请在执行该函数前执行 ESP_ERROR_CHECK( nvs_flash_init() );
 */
void campus_network_config(campus_network_config_t* can_cfg);


/* 校园网连接并且认证成功判断函数(内部实现为等待一个事件组) 
 * 参数为等待时间
 * 成功则返回非0值, 失败则返回0
 */
uint8_t campus_network_is_connected( TickType_t xTicksToWait);

#endif
