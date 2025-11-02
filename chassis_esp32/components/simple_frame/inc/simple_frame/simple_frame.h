#pragma once

#include "lwrb/lwrb.h"
#include <stdint.h>

typedef enum ParseState{
    NO_ERR,
    HEAD_BYTE_ERR,
    SIZE_BYTE_ERR,
    CHECK_SUM_ERR
}ParseState;

/*  数据格式:
 *  单字节包头 + 单字节包长度 + 可变多字节数据 + 单字节校验和

 *  其中:
 *  单字节包头      - 可以为任意字符或者单字节数字
 *  单字节包长度    - 整个包有多少个字节
 *  可变多字节数据  - 实际数据
 *  单字节校验和    - 整个包除了 `单字节校验和` 以外所有字节相加取低八位
 */
typedef struct Simple_Frame_Parser{
    //  数据包包头指定
    uint8_t frame_head_;
}Simple_Frame;

void sfp_init( Simple_Frame* sfp_, uint8_t frame_head);
void sfp_set_frame_head( Simple_Frame* sfp_, uint8_t frame_head);
uint8_t sfp_get_command( Simple_Frame* sfp_, lwrb_t* buff, uint8_t* data, uint16_t* size);
uint16_t sfp_create_frame( Simple_Frame* sfp_, uint8_t* frame, uint8_t* data,uint16_t data_size);
uint16_t sfp_pack_data( uint8_t* frame, uint8_t* data,uint16_t data_size, uint8_t frame_id);
ParseState sfp_unpack_data( uint8_t* frame, uint8_t* data ,uint16_t* data_size,uint8_t frame_id);