#include "user_network.h"
#include "campus_network.h"
#include "nvs_flash.h"
#include "task_priorities.h"
#include "task_period.h"
#include "esp_log.h"

static const char* TAG = "from -> "__FILE__;

static void network_init(void)
{

#ifdef CONFIG_USE_CAMPUS_NETWORK
    campus_network_config_t can_cfg = {
        .can_event_handler = NULL,
        .can_ssid = CONFIG_CAMPUS_NETWORK_SSID,
        .can_task_priority = NETWORK_TASK_PRIO,
        .can_user_id = CONFIG_CAMPUS_NETWORK_ID,
        .can_user_password = CONFIG_CAMPUS_NETWORK_PASSWORD,
    };
    campus_network_config( &can_cfg);
#else
    ESP_LOGW(TAG, "没有配置客户端模式, 不主动连接主机");
#endif


}


static void network_connect(void)
{

}

static void network_send_task(void* param)
{
    while(1)
    {

    }
    vTaskDelete(NULL);
}

static void network_receive_task(void* param)
{
    while(1)
    {
        
    }
    vTaskDelete(NULL);
}

void network_setup(void)
{
    /* 初始化 tcp ip协议栈( LWIP 实现 )任务 */
    ESP_ERROR_CHECK( nvs_flash_init() );

    /* 初始化网络配置 */
    network_init();

    /* 连接ROS2系统所在服务器 */
    network_connect();

    /* 创建收发任务持续通讯 */
    xTaskCreatePinnedToCore( network_send_task, "tcp_send_task", 8192, NULL, NETWORK_SEND_PRIO, NULL, 1);
    xTaskCreatePinnedToCore( network_receive_task, "tcp_receive_task", 8192, NULL, NETWORK_RECEIVE_PRIO, NULL, 1);
    ESP_LOGI(TAG, "network初始化成功, 收发任务执行中!!!");
}
