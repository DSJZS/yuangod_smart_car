#include "socket_ros2.h"
#include "task_priorities.h"
#include "task_period.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <sys/socket.h>

/* ROS2主机ip */
#define HOST_IP_ADDR    ( CONFIG_ROS2_SERVER_HOST_IP_ADDR )
/* ROS2主机对应进程端口 */
#define PORT            ( CONFIG_ROS2_SERVER_PORT )

static const char* TAG = "from -> "__FILE__;

/* 连接ROS2服务器的套接字 */
static int socket_ros2_tcp = -1;
/* 由底盘(chassis_esp32)工程的user_network直接移植过来
 * 即使本工程数据只发不收, 但为了避免因为修改而产生额外的bug(不是因为偷懒), 不删除读写锁
 */
static SemaphoreHandle_t socket_ros2_tcp_rwlock = NULL;

static void socket_ros2_tcp_close(void)
{
    xSemaphoreTakeRecursive( socket_ros2_tcp_rwlock , portMAX_DELAY);
    if ( socket_ros2_tcp != -1) {
        ESP_LOGE(TAG, "Shutting down socket_tcp...");
        shutdown( socket_ros2_tcp, 0);
        close( socket_ros2_tcp );
        socket_ros2_tcp = -1;
    }
    xSemaphoreGiveRecursive( socket_ros2_tcp_rwlock );
}

static uint8_t socket_ros2_tcp_connect()
{
    xSemaphoreTakeRecursive( socket_ros2_tcp_rwlock , portMAX_DELAY);

    struct sockaddr_in dest_addr;

    socket_ros2_tcp =  socket( AF_INET, SOCK_STREAM, 0);
    if (socket_ros2_tcp < 0) {
        ESP_LOGE(TAG, "无法创建socket: errno %d", errno);
        xSemaphoreGiveRecursive( socket_ros2_tcp_rwlock );
        return 0;
    }
    ESP_LOGI(TAG, "Socket tcp 连接, 正在连接 %s:%d", HOST_IP_ADDR, PORT);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    memset(&(dest_addr.sin_zero), 0, sizeof(dest_addr.sin_zero));

    int err = connect(socket_ros2_tcp, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket tcp 无法连接: errno %d", errno);
        socket_ros2_tcp_close( );
        xSemaphoreGiveRecursive( socket_ros2_tcp_rwlock );
        return 0;
    }
    ESP_LOGI(TAG, "Socket tcp 成功连接");
    xSemaphoreGiveRecursive( socket_ros2_tcp_rwlock );
    return 1;
}

static uint8_t socket_ros2_tcp_reconnect()
{
    socket_ros2_tcp_close();
    return socket_ros2_tcp_connect();
}

/* 配置socket */
static uint8_t socket_ros2_init(void)
{
    socket_ros2_tcp_rwlock = xSemaphoreCreateRecursiveMutex();
    return socket_ros2_tcp_connect();
}

void socket_ros2_tcp_send( uint8_t* tcp_tx_buffer, size_t buf_size)
{
    xSemaphoreTakeRecursive( socket_ros2_tcp_rwlock , portMAX_DELAY);
    int err = send( socket_ros2_tcp, tcp_tx_buffer, buf_size, 0);
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        socket_ros2_tcp_reconnect();
    }
    xSemaphoreGiveRecursive( socket_ros2_tcp_rwlock );
}

void socket_ros2_config(void)
{
    uint8_t ret = 0;

    /* 初始化socket */
    ret = socket_ros2_init();
    if( ret == 0 )
        ESP_LOGI(TAG, "socket 初始化失败!!!");
    else
        ESP_LOGI(TAG, "socket 初始化成功!!!");
}
