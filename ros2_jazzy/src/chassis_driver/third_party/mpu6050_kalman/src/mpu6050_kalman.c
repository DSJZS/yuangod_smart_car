/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include "mpu6050_kalman/mpu6050_kalman.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295

static Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

static Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

static Kalman_t KalmanZ = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

/* 静态下的校准, 动态性能勉强凑合着用 */
static const double static_yaw_corrector = 0.035;

/**
  * @brief MPU6050 卡尔曼滤波( 角度制 )
  *
  * @param DataStruct   存放滤波数据的结构体
  * @param gx_2_az      MPU6050六轴数据
  * @param dt           距离上次滤波的时间间隔(单位:s)
  */
void mpu6050_kalman_filter_deg( MPU6050_Data_t *DataStruct,float gx, float gy, float gz, float ax, float ay, float az, double dt)
{
    DataStruct->Ax = ax;
    DataStruct->Ay = ay;
    DataStruct->Az = az;
    DataStruct->Gx = gx;
    DataStruct->Gy = gy;
    DataStruct->Gz = gz;
    DataStruct->dt = dt;
    MPU6050_Read_All(DataStruct);
}

/**
  * @brief MPU6050 卡尔曼滤波( 弧度制 )
  *
  * @param DataStruct   存放滤波数据的结构体
  * @param gx_2_az      MPU6050六轴数据
  * @param dt           距离上次滤波的时间间隔(单位:s)
  */
void mpu6050_kalman_filter_rad( MPU6050_Data_t *DataStruct,float gx, float gy, float gz, float ax, float ay, float az, double dt)
{
    mpu6050_kalman_filter_deg( DataStruct, gx,  gy,  gz,  ax,  ay,  az,  dt);
    DataStruct->KalmanAngleX *= DEG_TO_RAD;
    DataStruct->KalmanAngleY *= DEG_TO_RAD;
    DataStruct->KalmanAngleZ *= DEG_TO_RAD;
}

/**
  * @brief 估计欧拉角(角度制)
  *
  * @param DataStruct   存放滤波数据的结构体
  */
void MPU6050_Read_All(MPU6050_Data_t *DataStruct)
{
    /* DataStruct 需要 xyz 六轴数据 dt 数据 */
    /* DataStruct->KalmanAngleX DataStruct->KalmanAngleY 获取滤波后的数据*/

    // Kalman angle solve
    double roll;
    double roll_sqrt = sqrt( DataStruct->Ax * DataStruct->Ax + DataStruct->Az * DataStruct->Az);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Ay / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }

    double pitch = atan2(-DataStruct->Ax, DataStruct->Az) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, DataStruct->dt);
    }

    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, DataStruct->dt);

    /* 对我而言XY轴好像反了，这里取反 */
    DataStruct->KalmanAngleX = -DataStruct->KalmanAngleX;
    DataStruct->KalmanAngleY = -DataStruct->KalmanAngleY;
    // 对于Z轴，我们使用陀螺仪积分，因为没有磁场传感器
    // 注意：这会随时间漂移，但在短时间内是准确的
    static double yaw = 0.0;
    yaw += DataStruct->Gz * DataStruct->dt - static_yaw_corrector;
    // 限制角度范围在-180到180度之间
    while (yaw > 180.0) yaw -= 360.0;
    while (yaw < -180.0) yaw += 360.0;
    DataStruct->KalmanAngleZ = Kalman_getAngle(&KalmanZ, yaw, DataStruct->Gz, DataStruct->dt);
    // 限制Z轴角度范围
    while (DataStruct->KalmanAngleZ > 180.0) DataStruct->KalmanAngleZ -= 360.0;
    while (DataStruct->KalmanAngleZ < -180.0) DataStruct->KalmanAngleZ += 360.0;
}

/**
  * @brief 针对角度的卡尔曼滤波
  *
  * @param Kalman   卡尔曼滤波结构体
  * @param newAngle 新的测量值
  * @param newRate  新的测量值的速度( 也可以说速率 )
  * @param dt       距离上次滤波的时间间隔(单位:s)
  * @return
  *      - kalman_angle: 卡尔曼滤波得到的滤波值
  */
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}
