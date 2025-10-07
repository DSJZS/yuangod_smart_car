#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "from -> "__FILE__;

void hello_world_task(void* param)
{
    const TickType_t xDelay500ms = pdMS_TO_TICKS( 500UL );
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1){
        ESP_LOGI(TAG, "小车开始初始化!");
        vTaskDelayUntil(&xLastWakeTime, xDelay500ms);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "小车开始初始化!");
    xTaskCreatePinnedToCore( hello_world_task, "hello_world", 2048, NULL, 3, NULL, 1);
}
