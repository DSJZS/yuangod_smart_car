#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "user_network/user_network.h"
#include "user_motors/user_motors.h"
#include "user_imu/user_imu.h"
#include "user_battery/user_battery.h"

static const char* TAG = "from -> "__FILE__;

void hello_user_i_am_yuangod(void)
{
    ESP_LOGI(TAG,   "\n"
                    "___  _ _     ____  _        _____ ____  ____ \n"
                    "\\  \\/// \\ /\\/  _ \\/ \\  /|  /  __//  _ \\/  _ \\\n"
                    " \\  / | | ||| / \\|| |\\ ||  | |  _| / \\|| | \\|\n"
                    " / /  | \\_/|| |-||| | \\||  | |_//| \\_/|| |_/|\n"
                    "/_/   \\____/\\_/ \\|\\_/  \\|  \\____\\____/\\____/\n");
}

void app_main(void)
{
    hello_user_i_am_yuangod();
    ESP_LOGI(TAG, "小车开始初始化!");

    imu_init();
    motor_init();   //  pid未完成
    battery_estimation_init();  //  电压测量不正确

    network_setup();
    
    ESP_LOGI(TAG, "小车初始化完毕!");


    const TickType_t xDelay500ms = pdMS_TO_TICKS( 500UL );
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1) {

        // float capacity = 0.0f;
        // battery_get_capacity(&capacity);
        // ESP_LOGI(TAG, "capacity: %.4f", capacity);

        vTaskDelayUntil(&xLastWakeTime, xDelay500ms);
    }
}
