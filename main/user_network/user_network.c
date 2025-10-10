#include "user_network.h"
#include "campus_network.h"
#include "nvs_flash.h"
#include "task_priorities.h"

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
#endif
}

static void network_connect(void)
{

}

void network_setup(void)
{
    /* 初始化 tcp ip协议栈( LWIP 实现 )任务 */
    ESP_ERROR_CHECK( nvs_flash_init() );

    /* 初始化网络配置 */
    network_init();

    /* 连接ROS2系统所在服务器 */
    network_connect();
}