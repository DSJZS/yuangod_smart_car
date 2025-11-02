/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#ifdef __cplusplus 
extern "C" {
#endif  //__cplusplus

#include <stdint.h>

// MPU6050 structure
typedef struct
{
    double Ax;
    double Ay;
    double Az;

    double Gx;
    double Gy;
    double Gz;

    double dt;  //s

    double KalmanAngleX;
    double KalmanAngleY;
    double KalmanAngleZ;
} MPU6050_Data_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

void mpu6050_kalman_filter_deg( MPU6050_Data_t *DataStruct,float gx, float gy, float gz, float ax, float ay, float az, double dt);
void mpu6050_kalman_filter_rad( MPU6050_Data_t *DataStruct,float gx, float gy, float gz, float ax, float ay, float az, double dt);

void MPU6050_Read_All( MPU6050_Data_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#ifdef __cplusplus
}
#endif  //__cplusplus

#endif /* INC_GY521_H_ */
