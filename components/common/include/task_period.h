#ifndef __YG_COMMON_TASK_PERIOD_H__
#define __YG_COMMON_TASK_PERIOD_H__ 

/*********************************** 任务周期配置 ***********************************/

/* 当前配置 CONFIG_FREERTOS_HZ = 1000 */
/* 下述的周期单位固定为 ms */

/* IMU数据读取任务周期 */
#define IMU_TASK_PERIOD         ( 100 )

/* 电机控制周期 */
#define MOTOR_TASK_PERIOD       ( 50 )

/* 底盘数据TCP发送周期 */
#define TCP_SEND_TASK_PERIOD    ( 50 )

/* 底盘数据TCP接受周期 */
#define TCP_RECEIVE_TASK_PERIOD ( 1 )

/*********************************** 任务周期配置 ***********************************/

#endif
