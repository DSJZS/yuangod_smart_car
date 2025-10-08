#ifndef __USER_MOTOR_H__
#define __USER_MOTOR_H__

#include "bdc_motor_control.h"

extern dc_motor_control_context_t left_motor_dev;
extern dc_motor_control_context_t right_motor_dev;

void motor_init(void);
void motor_set_target( dc_motor_control_context_t* control_context, int target_pulses);

#endif
