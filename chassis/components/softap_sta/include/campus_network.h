#ifndef __CAMPUS_NETWORK_H__
#define __CAMPUS_NETWORK_H__

#include <stdint.h>

/* 校园网登陆函数
 * 需要实现配置好STA模式相关功能
 */
uint8_t campus_network_login( char* user_id, char* user_password);

#endif
