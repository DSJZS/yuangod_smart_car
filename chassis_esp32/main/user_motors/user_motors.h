#ifndef __USER_MOTOR_H__
#define __USER_MOTOR_H__

#include "bdc_motor_control.h"


void motor_init(void);
void motor_set_speed( float linear_x, float angular_z);
void motor_get_speed( float* linear_x, float* angular_z);
void motor_set_raw_speed( float left_speed, float right_speed);
void motor_get_raw_speed( float* left_speed, float* right_speed);

#endif
