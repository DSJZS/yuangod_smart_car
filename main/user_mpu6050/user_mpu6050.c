#include "user_mpu6050,h"
#include "driver/gpio.h"

static mpu6050_handle_t mpu6050_dev;

void mpu6050_init(void)
{
    mpu6050_i2c_config_t i2c_config = {
        .port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_37,
        .scl_io_num = GPIO_NUM_38,
    };
    mpu6050_dev = mpu6050_create( &i2c_config, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050_dev, ACCE_FS_2G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_dev);
}

void mpu6050_read( mpu6050_acce_value_t *const acce, mpu6050_gyro_value_t *const gyro)
{
    mpu6050_get_acce(mpu6050_dev, acce);
    mpu6050_get_gyro(mpu6050_dev, gyro);
    // mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);
}

void mpu6050_get_rp( mpu6050_acce_value_t *const acce, mpu6050_gyro_value_t *const gyro, complimentary_angle_t *const complimentary_angle)
{
    mpu6050_complimentory_filter( mpu6050_dev, acce, gyro, complimentary_angle);
}
