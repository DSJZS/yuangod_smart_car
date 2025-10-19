#include "user_network.h"
#include "task_priorities.h"
#include "task_period.h"
#include "esp_log.h"
#include "softap_sta.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "from -> "__FILE__;

static void network_init(void)
{
    softap_sta_config();
}


static void network_connect(void)
{

}

static void network_send_task(void* param)
{
    const TickType_t xDelay = pdMS_TO_TICKS( TCP_SEND_TASK_PERIOD );
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
    vTaskDelete(NULL);
}

static void network_receive_task(void* param)
{
    const TickType_t xDelay = pdMS_TO_TICKS( TCP_RECEIVE_TASK_PERIOD );
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
    vTaskDelete(NULL);
}

void network_setup(void)
{
    /* 初始化网络配置 */
    network_init();

    /* 连接ROS2系统所在服务器 */
    network_connect();

    /* 创建收发任务持续通讯 */
    xTaskCreatePinnedToCore( network_send_task, "yuangod_tcp_send_task", 8192, NULL, NETWORK_SEND_PRIO, NULL, 1);
    xTaskCreatePinnedToCore( network_receive_task, "yuangod_tcp_receive_task", 8192, NULL, NETWORK_RECEIVE_PRIO, NULL, 1);
    ESP_LOGI(TAG, "network初始化成功, 收发任务执行中!!!");
}
