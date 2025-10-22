#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_station.h"

void app_main(void)
{
    ESP_LOGI("TAG", "hello world!!");

    wifi_station_init();
    while(1)
    {
        vTaskDelay(1);
    }
}
