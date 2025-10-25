#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_station.h"
#include "socket_ros2.h"
#include "ld14.h"

void app_main(void)
{
    ESP_LOGI("TAG", "hello world!!");

    /* wifi站点模式初始化用于连接底盘上的ESP32 AP */
    wifi_station_init();
    /* 创建连接ROS2服务器的socket */
    socket_ros2_config();
    /* 初始化连接LD14的串口并创建一个任务定时转发雷达数据到ROS2服务器 */
    ld14_init();
}
