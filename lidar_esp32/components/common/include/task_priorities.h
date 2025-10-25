#ifndef __YG_COMMON_TASK_PRIORITIES_H__
#define __YG_COMMON_TASK_PRIORITIES_H__

/************************************ 优先级配置 ************************************/
 
/* ESP32默认配置的最大可支持优先级为 25 */

/* 本项目用户优先级分为7个等级具体如下，任务优先级的安排只能是这七个，不允许 Magic Number 的出现 */
/* 极高优先级 */
#define TASK_PRIORITY_CRITICAL     ( TASK_PRIORITY_HIGH + 1 )       
/* 高优先级 */
#define TASK_PRIORITY_HIGH         ( TASK_PRIORITY_ABOVE_NORMAL + 1 )        
/* 偏高优先级 */
#define TASK_PRIORITY_ABOVE_NORMAL ( TASK_PRIORITY_NORMAL + 1 )       
/* 中等优先级 */
#define TASK_PRIORITY_NORMAL       ( TASK_PRIORITY_BELOW_NORMAL + 1 )       
/* 偏低优先级 */
#define TASK_PRIORITY_BELOW_NORMAL ( TASK_PRIORITY_LOW + 1 )    
/* 低优先级 */
#define TASK_PRIORITY_LOW          ( TASK_PRIORITY_LOWEST + 1 )    
/* 极低优先级 */
#define TASK_PRIORITY_LOWEST       ( 3 )    

/* 具体任务的配置, 宏定义的文本上下顺序应该与优先级的顺序一致 */

/* 激光雷达任务优先级 */
#define LIDAR_TASK_PRIO   ( TASK_PRIORITY_CRITICAL )


/************************************ 优先级配置 ************************************/

#endif
