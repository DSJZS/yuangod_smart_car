#include "user_imu.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "task_period.h"
#include "task_priorities.h"

static const char* TAG = "from -> "__FILE__;

static mpu6050_handle_t mpu6050_dev = NULL;
static QueueHandle_t imu_mailbox = NULL;

typedef struct {
    mpu6050_acce_value_t acce_value;
    mpu6050_gyro_value_t gyro_value;
} imu_data_t;

/* 外界根据原始数据计算 roll 与 pitch */
void imu_get_rp( mpu6050_acce_value_t *const acce_value, mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
{
    mpu6050_complimentory_filter( mpu6050_dev, acce_value, gyro_value, complimentary_angle);
}

/* 外界读取加速度与陀螺仪的原始数据 */
void imu_get_data( mpu6050_acce_value_t *const acce_value, mpu6050_gyro_value_t *const gyro_value)
{
    imu_data_t imu_data = {0};
    xQueuePeek( imu_mailbox, &imu_data, portMAX_DELAY);
    memcpy( acce_value, &( imu_data.acce_value ), sizeof( mpu6050_acce_value_t ));
    memcpy( gyro_value, &( imu_data.gyro_value ), sizeof( mpu6050_gyro_value_t ));
}

/* 读取加速度与陀螺仪的原始数据 */
static void mpu6050_read( mpu6050_acce_value_t *const acce_value, mpu6050_gyro_value_t *const gyro_value)
{
    mpu6050_get_acce(mpu6050_dev, acce_value);
    mpu6050_get_gyro(mpu6050_dev, gyro_value);
}

void imu_task(void* param)
{
    const TickType_t xDelay = pdMS_TO_TICKS( IMU_TASK_PERIOD );
    TickType_t xLastWakeTime = xTaskGetTickCount();

    mpu6050_handle_t* dev = (mpu6050_handle_t*)param;
    imu_data_t imu_data = {0};
    while(1)
    {
        mpu6050_read( &(imu_data.acce_value), &(imu_data.gyro_value) );
        xQueueOverwrite( imu_mailbox, &imu_data);
        ESP_LOGD(TAG, "ax=%f,ay=%f,az=%f,gx=%f,gy=%f,gz=%f", 
            imu_data.acce_value.acce_x, imu_data.acce_value.acce_y, imu_data.acce_value.acce_z,
            imu_data.gyro_value.gyro_x, imu_data.gyro_value.gyro_y, imu_data.gyro_value.gyro_z);
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }

    mpu6050_delete( *dev );
    *dev = NULL;
    vQueueDelete(imu_mailbox);
    imu_mailbox = NULL;
    vTaskDelete(NULL);
}

void imu_init(void)
{
    mpu6050_i2c_config_t i2c_config = {
        .port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_37,
        .scl_io_num = GPIO_NUM_38,
    };
    mpu6050_dev = mpu6050_create( &i2c_config, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050_dev, ACCE_FS_2G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_dev);

    imu_mailbox = xQueueCreate( 1, sizeof(imu_data_t) );
    xTaskCreatePinnedToCore( imu_task, "imu_task", 4096, &mpu6050_dev, IMU_TASK_PRIO, NULL, 1);

    ESP_LOGI(TAG, "IMU 初始化完毕!!!");
}

