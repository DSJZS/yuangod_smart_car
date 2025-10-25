#ifndef __SOCKET_CONFIG_H__
#define __SOCKET_CONFIG_H__

#include <stdint.h>
#include <stddef.h>

void socket_ros2_config(void);
void socket_ros2_tcp_send( uint8_t* tcp_tx_buffer, size_t buf_size);

#endif
