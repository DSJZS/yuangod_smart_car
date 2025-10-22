#include "user_network.h"
#include "task_priorities.h"
#include "task_period.h"
#include "esp_log.h"
#include "softap_sta.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <sys/socket.h>
#include "user_imu/user_imu.h"
#include "user_battery/user_battery.h"
#include "user_motors/user_motors.h"

/* ROS2主机ip */
#define HOST_IP_ADDR    ( CONFIG_ROS2_SERVER_HOST_IP_ADDR )
/* ROS2主机对应进程端口 */
#define PORT            ( CONFIG_ROS2_SERVER_PORT )

static const char* TAG = "from -> "__FILE__;
static int socket_ros2_tcp = -1;

/* 发送数据结构体 */
typedef struct {
    uint8_t head;                   //  1
    float x_speed;                  //  4
    float y_speed;                  //  4
    float z_speed;                  //  4
    float battery_capacity;         //  4
    mpu6050_acce_value_t imu_acce;  //  3*4=12
    mpu6050_gyro_value_t imu_gyro;  //  3*4=12
    uint8_t tail;                   //  1
} send_data_t;                      //  sum = TCP_BUFFER_LEN - 1
// TCP传输buffer长度, 加一位为校验位
#define TCP_BUFFER_LEN     ( (1+4+4+4+4+12+12+1) + 1 ) 
static uint8_t tcp_buffer[TCP_BUFFER_LEN] = {0};

/* 结构体收集信息 */
static void get_car_data( send_data_t* data)
{
    float x=0,y=0,z=0;   //  临时变量
    float battery=0;        //  临时变量
    data->head = 0x55;      //  bin: 0101 0101
    motor_get_speed( &x, &z);
    data->x_speed = x;
    data->y_speed = y;
    data->z_speed = z;
    battery_get_capacity(&battery);
    data->battery_capacity = battery;
    imu_get_data( &( data->imu_acce ), &( data->imu_gyro ) );
    data->tail = 0xAA;      //  bin: 1010 1010
}

/**
  * @brief 计算发送或接受的数据校验
  *
  * @param buffer 计算校验和的缓存
  * @param count_number 校验前多少位
  * @return
  *      - check_sum: 计算
  */
uint8_t buffer_check_sum(uint8_t* buffer, uint8_t count_number)
{
	uint8_t check_sum = 0, k = 0;

    for( k = 0 ; k < count_number ; k++ )
    {
        check_sum=check_sum^buffer[k];
    }

	return check_sum;
}

/* 结构体数据转为字节流(序列化) */
static void serialize_car_data( uint8_t* buffer, send_data_t* data)
{
    uint16_t index = 0, size = 0;

    size = sizeof( data->head );
    memcpy( &( buffer[index] ), &(data->head), size);
    index += size;

    size = sizeof( data->x_speed );
    memcpy( &( buffer[index] ), &(data->x_speed), size);
    index += size;

    size = sizeof( data->y_speed );
    memcpy( &( buffer[index] ), &(data->y_speed), size);
    index += size;

    size = sizeof( data->z_speed );
    memcpy( &( buffer[index] ), &(data->z_speed), size);
    index += size;

    size = sizeof( data->imu_acce.acce_x );
    memcpy( &( buffer[index] ), &(data->imu_acce.acce_x), size);
    index += size;

    size = sizeof( data->imu_acce.acce_y );
    memcpy( &( buffer[index] ), &(data->imu_acce.acce_y), size);
    index += size;

    size = sizeof( data->imu_acce.acce_z );
    memcpy( &( buffer[index] ), &(data->imu_acce.acce_z), size);
    index += size;

    size = sizeof( data->imu_gyro.gyro_x );
    memcpy( &( buffer[index] ), &(data->imu_gyro.gyro_x), size);
    index += size;

    size = sizeof( data->imu_gyro.gyro_y );
    memcpy( &( buffer[index] ), &(data->imu_gyro.gyro_y), size);
    index += size;

    size = sizeof( data->imu_gyro.gyro_z );
    memcpy( &( buffer[index] ), &(data->imu_gyro.gyro_z), size);
    index += size;

    size = sizeof( data->battery_capacity );
    memcpy( &( buffer[index] ), &(data->battery_capacity), size);
    index += size;

    uint8_t check_sum = buffer_check_sum( buffer, index);
    size = sizeof( check_sum );
    memcpy( &( buffer[index] ), &(check_sum), size);
    index += size;

    size = sizeof( data->tail );
    memcpy( &( buffer[index] ), &(data->tail), size);
    index += size;
}

/* 配置softap_sta */
static uint8_t network_config(void)
{
    return softap_sta_config();
}

static void network_socket_tcp_close(int* socket_tcp)
{
    int socket = *socket_tcp;
    if ( socket != -1) {
        ESP_LOGE(TAG, "Shutting down socket_tcp...");
        shutdown( socket, 0);
        close( socket );
        *socket_tcp = -1;
    }
}
/* 配置socket */
static uint8_t network_socket_init(void)
{
    struct sockaddr_in dest_addr;

    network_socket_tcp_close( &socket_ros2_tcp );

    socket_ros2_tcp =  socket( AF_INET, SOCK_STREAM, 0);
    if (socket_ros2_tcp < 0) {
        ESP_LOGE(TAG, "无法创建socket: errno %d", errno);
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
        network_socket_tcp_close( &socket_ros2_tcp );
        return 0;
    }
    ESP_LOGI(TAG, "Socket tcp 成功连接");

    return 1;
}

static void network_send_task(void* param)
{
    const TickType_t xDelay = pdMS_TO_TICKS( TCP_SEND_TASK_PERIOD );
    TickType_t xLastWakeTime = xTaskGetTickCount();

    send_data_t data = { 0 };
    int err = 0;
    while(1)
    {
        get_car_data( &data);
        serialize_car_data( tcp_buffer, &data);
        // ESP_LOGI(TAG, "%x",tcp_buffer[42]);

        err = send( socket_ros2_tcp, tcp_buffer, sizeof( tcp_buffer ), 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            // break;
        }
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
    uint8_t ret = 0;
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
