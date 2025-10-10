#ifndef __USER_IMU_H__
#define __USER_IMU_H__

#include "mpu6050.h"

void imu_init(void);
void imu_get_data( mpu6050_acce_value_t *const acce_value, mpu6050_gyro_value_t *const gyro_value);
void imu_get_rp( mpu6050_acce_value_t *const acce_value, mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle);

#endif
