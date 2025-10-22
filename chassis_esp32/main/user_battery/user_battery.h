#ifndef __USER_BATTERY_H__
#define __USER_BATTERY_H__

void battery_estimation_init(void);
/* 获取电量的百分比 */
void battery_get_capacity(float *capacity);

#endif
