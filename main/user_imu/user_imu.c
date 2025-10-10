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
static mpu6050_acce_value_t s_acce = {0};
static mpu6050_gyro_value_t s_gyro = {0};
static SemaphoreHandle_t imu_mutex = NULL;

/* 外界根据原始数据计算 roll 与 pitch */
void imu_get_rp( mpu6050_acce_value_t *const acce_value, mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
{
    mpu6050_complimentory_filter( mpu6050_dev, acce_value, gyro_value, complimentary_angle);
}

/* 外界读取加速度与陀螺仪的原始数据 */
void imu_get_data( mpu6050_acce_value_t *const acce_value, mpu6050_gyro_value_t *const gyro_value)
{
    xSemaphoreTake( imu_mutex, portMAX_DELAY);
    memcpy( acce_value, &s_acce, sizeof( mpu6050_acce_value_t ));
    memcpy( gyro_value, &s_gyro, sizeof( mpu6050_gyro_value_t ));
    xSemaphoreGive( imu_mutex );
}

/* 读取加速度与陀螺仪的原始数据 */
static void mpu6050_read( mpu6050_acce_value_t *const acce_value, mpu6050_gyro_value_t *const gyro_value)
{
    xSemaphoreTake( imu_mutex, portMAX_DELAY);
    mpu6050_get_acce(mpu6050_dev, acce_value);
    mpu6050_get_gyro(mpu6050_dev, gyro_value);
    xSemaphoreGive( imu_mutex );
}

void imu_task(void* param)
{
    const TickType_t xDelay = pdMS_TO_TICKS( IMU_TASK_PERIOD );
    TickType_t xLastWakeTime = xTaskGetTickCount();

    mpu6050_handle_t* dev = (mpu6050_handle_t*)param;
    while(1)
    {
        mpu6050_read( &s_acce, &s_gyro);
        ESP_LOGI(TAG, "ax=%f,ay=%f,az=%f,gx=%f,gy=%f,gz=%f", s_acce.acce_x,s_acce.acce_y,s_acce.acce_z,s_gyro.gyro_x,s_gyro.gyro_y,s_gyro.gyro_z);
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }

    mpu6050_delete( *dev );
    *dev = NULL;
    vSemaphoreDelete(imu_mutex);
    imu_mutex = NULL;
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

    imu_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore( imu_task, "imu_task", 4096, &mpu6050_dev, IMU_TASK_PRIO, NULL, 1);
}

