#include "ld14.h"
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "socket_ros2.h"
#include "task_period.h"
#include "task_priorities.h"

const char * TAG = "ld14";

#define LD14_UART_NUM   (CONFIG_LD14_UART_NUM)
#define LD14_TXD_PIN    (CONFIG_LD14_UART_TXD)
#define LD14_RXD_PIN    (CONFIG_LD14_UART_RXD)

#define LD14_BUF_SIZE (1024)

/**
  * @brief 串口配置函数, 本项目中主要为LD14组件服务
  */
static void ld14_uart_config(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = CONFIG_LD14_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //安装UART驱动, 并且获得一个队列, 底层相关中断遇到一些情况时会将event放入队列以唤醒相关task ( 事件驱动 )
    ESP_ERROR_CHECK(uart_driver_install(LD14_UART_NUM, LD14_BUF_SIZE * 2, 0, 0, NULL, 0));
    uart_param_config(LD14_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(LD14_UART_NUM, LD14_TXD_PIN, LD14_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "串口初始化完毕!!!");
}

/**
  * @brief 串口读取函数, 本项目中主要为LD14组件服务
  *
  * @param rx_buffer 用户接收缓存区指针
  * @param buf_size 用户接收缓存区长度
  * @return
  *      - copy_len:    >=  0 时表示从串口RX缓存中读取的字节数； 
  *                     == -1 时表示出现了错误
  */
static int ld14_uart_read(uint8_t* rx_buffer, size_t buf_size)
{
    return uart_read_bytes(LD14_UART_NUM, rx_buffer, buf_size, pdMS_TO_TICKS(0));
}

static void ld14_task(void* param)
{
    uint8_t rx_buffer[256] = {0};
    const size_t buf_size = 256;
    int copy_len = 0;   //  注意这里类型应该为 int, 而非 size_t, 因为要通过该变量是否为负数来判断串口接收是否出错

    while(1)
    {
        copy_len = ld14_uart_read( rx_buffer, buf_size);
        if( copy_len >= 0 ) {
            /*  运输层协议的选择:
                    esp32 使用 lwip TCP/IP协议栈，可以使用 TCP 或者 UDP 传输数据到ROS2服务器,
                    但是基于以下的原因，本项目的雷达数据选择 TCP协议 传输( 和原项目一样,即使原项目注释写着 UDP ):
                        1. LD14这款雷达产生的数据量不大, 实时性要求较为容易满足。
                        2. 本项目涉及建图、导航等功能, 要确保可靠的连接与传输(即做到"esp32传什么, ros2服务器就收到什么")。

                esp32转发前是否需要处理 uart 接收到的流数据:
                    基于以下的原因, esp32只负责转发流数据, 处理交由ROS2服务器处理:
                        1. esp32 由于算力等限制无法完成 SLAM 和 导航等操作,
                            即使处理流数据为格式化数据, 也"来不及看和算"。
                        2. TCP 是流式传输, 不保留消息边界, 即使esp32发送格式化数据, ros2收到的数据也没有格式,
                            故直接转发流数据效率更高。  */
            socket_ros2_tcp_send( rx_buffer, copy_len);
        }
        
        /* 延时的目的是确保有时间给看门狗定时器复位避免看门狗RST, 故这个延时时间应该尽可能的低 */
        vTaskDelay(pdMS_TO_TICKS(LIDAR_TASK_PERIOD));
    }
    vTaskDelete(NULL);
}

void ld14_init(void)
{
    /* 初始化LD14使用的串口 */
    ld14_uart_config();
    
    /* 创建 ld14_task 之前 socket 的初始化必须完成(当前在 app_main 中已初始化) */
    xTaskCreate(ld14_task, "ld14_task", 4096 * 8, NULL, LIDAR_TASK_PRIO, NULL);
    ESP_LOGI(TAG, "激光雷达初始化完毕!!!");
}
