#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    ESP_LOGI("TAG", "hello world!!");
    while(1)
    {
        vTaskDelay(1);
    }
}
