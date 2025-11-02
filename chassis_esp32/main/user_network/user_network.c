#include "user_network.h"
#include "task_priorities.h"
#include "task_period.h"
#include "esp_log.h"
#include "softap_sta.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "simple_frame/simple_frame.h"

#include <sys/socket.h>
#include "user_imu/user_imu.h"
#include "user_battery/user_battery.h"
#include "user_motors/user_motors.h"

/* ROS2主机ip */
#define HOST_IP_ADDR    ( CONFIG_ROS2_SERVER_HOST_IP_ADDR )
/* ROS2主机对应进程端口 */
#define PORT            ( CONFIG_ROS2_SERVER_PORT )

static const char* TAG = "from -> "__FILE__;

/* 连接ROS2服务器的套接字 */
static int socket_ros2_tcp = -1;
/* 管理连接ROS2服务器的套接字的读写的递归锁 
 * 虽然TCP是全双工不需要互斥锁处理，但是考虑到重连接时的线程安全，还是添加了递归锁管理
 */
static SemaphoreHandle_t socket_ros2_tcp_rwlock = NULL;

/* 发送数据结构体 */
typedef struct {
    float x_speed;                  //  4
    float y_speed;                  //  4
    float z_speed;                  //  4
    float battery_capacity;         //  4
    mpu6050_acce_value_t imu_acce;  //  3*4=12
    mpu6050_gyro_value_t imu_gyro;  //  3*4=12
} send_data_t;                      
/* TCP传输buffer长度 */
#define TCP_BUFFER_LEN      ( 4+4+4+4+12+12 )
/* Simple Frame Parser */
#define SFP_FRAME_HEAD      ( 0xAA )
static Simple_Frame sfp;

/* 结构体收集信息 */
static void get_car_data( send_data_t* data)
{
    float x=0,y=0,z=0;   //  临时变量
    float battery=0;        //  临时变量     
    motor_get_speed( &x, &z);
    data->x_speed = x;
    data->y_speed = y;
    data->z_speed = z;
    battery_get_capacity(&battery);
    data->battery_capacity = battery;
    imu_get_data( &( data->imu_acce ), &( data->imu_gyro ) );
}

/* 结构体数据转为字节流(序列化) */
static void serialize_car_data( uint8_t* buffer, send_data_t* data)
{
    uint16_t index = 0, copied = 0;

    #define serialization(var) copied = sizeof( var ); memcpy( &( buffer[index] ), &var, copied); index += copied;
    serialization( data->x_speed );
    serialization( data->y_speed );
    serialization( data->z_speed );
    serialization( data->imu_acce.acce_x );
    serialization( data->imu_acce.acce_y );
    serialization( data->imu_acce.acce_z );
    serialization( data->imu_gyro.gyro_x );
    serialization( data->imu_gyro.gyro_y );
    serialization( data->imu_gyro.gyro_z );
    serialization( data->battery_capacity );
    #undef serialization
}

/* 配置softap_sta */
static uint8_t network_config(void)
{
    return softap_sta_config();
}

static void network_socket_tcp_close(void)
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

static uint8_t network_socket_tcp_connect()
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
        network_socket_tcp_close( );
        xSemaphoreGiveRecursive( socket_ros2_tcp_rwlock );
        return 0;
    }
    ESP_LOGI(TAG, "Socket tcp 成功连接");
    xSemaphoreGiveRecursive( socket_ros2_tcp_rwlock );
    return 1;
}

static uint8_t network_socket_tcp_reconnect()
{
    network_socket_tcp_close();
    return network_socket_tcp_connect();
}

/* 配置socket */
static uint8_t network_socket_init(void)
{
    socket_ros2_tcp_rwlock = xSemaphoreCreateRecursiveMutex();
    return network_socket_tcp_connect( );
}

static void network_send_task(void* param)
{
    const TickType_t xDelay = pdMS_TO_TICKS( TCP_SEND_TASK_PERIOD );
    TickType_t xLastWakeTime = xTaskGetTickCount();

    send_data_t data = { 0 };
    uint8_t frame_buffer[TCP_BUFFER_LEN+3] = {0};
    int err = 0;
    while(1)
    {
        uint8_t data_buffer[TCP_BUFFER_LEN] = {0};
        uint16_t frame_size = 0;
        get_car_data( &data);
        serialize_car_data( data_buffer, &data);
        frame_size = sfp_create_frame( &sfp, frame_buffer, data_buffer, sizeof( data_buffer ));

        xSemaphoreTakeRecursive( socket_ros2_tcp_rwlock , portMAX_DELAY);
        err = send( socket_ros2_tcp, frame_buffer, frame_size, 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            network_socket_tcp_reconnect();
        }
        xSemaphoreGiveRecursive( socket_ros2_tcp_rwlock );

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
        xSemaphoreTakeRecursive( socket_ros2_tcp_rwlock , portMAX_DELAY);

        xSemaphoreGiveRecursive( socket_ros2_tcp_rwlock );

        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
    vTaskDelete(NULL);
}

void network_setup(void)
{
    uint8_t ret = 0;

    /* 初始化 Simple Frame Parser */
    sfp_init( &sfp, SFP_FRAME_HEAD);

    /* 初始化网络配置 */
    ret = network_config();
    if( ret == 0 )
        ESP_LOGI(TAG, "网络配置初始化失败");

    /* 初始化socket */
    ret = network_socket_init();
    if( ret == 0 )
        ESP_LOGI(TAG, "socket 初始化失败");

    /* 创建收发任务持续通讯 */
    xTaskCreatePinnedToCore( network_send_task, "yuangod_tcp_send_task", 8192, NULL, NETWORK_SEND_PRIO, NULL, 1);
    xTaskCreatePinnedToCore( network_receive_task, "yuangod_tcp_receive_task", 8192, NULL, NETWORK_RECEIVE_PRIO, NULL, 1);
    ESP_LOGI(TAG, "network初始化成功, 收发任务执行中!!!");
}
