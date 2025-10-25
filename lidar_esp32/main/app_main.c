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

    // wifi站点模式初始化用于连接底盘上的ESP32 AP
    wifi_station_init();
    socket_ros2_config();

    ld14_init();

    while(1)
    {
        vTaskDelay(1);
    }
}
