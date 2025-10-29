#pragma once

#include "lwrb/lwrb.h"
#include <stdint.h>

namespace cya::hal::protocol
{

/*  数据格式:
 *  单字节包头 + 单字节包长度 + 可变多字节数据 + 单字节校验和

 *  其中:
 *  单字节包头      - 可以为任意字符或者单字节数字
 *  单字节包长度    - 整个包有多少个字节
 *  可变多字节数据  - 实际数据
 *  单字节校验和    - 整个包除了 `单字节校验和` 以外所有字节相加取低八位
 */
typedef class Simple_Frame_Parser{
private:
    enum ParseState{
        NO_ERR,
        HEAD_BYTE_ERR,
        SIZE_BYTE_ERR,
        CHECK_SUM_ERR
    };
    //  数据包最小长度
    constexpr static uint16_t kCommandMinLength = 4;
    //  数据包解析缓存区最大长度
    constexpr static uint16_t kParserBufferLength = 512;
    //  数据包包头指定
    uint8_t frame_head_;
public:
    Simple_Frame_Parser( uint8_t frame_head );
    void set_frame_head( uint8_t frame_head );

    bool get_command( lwrb_t* buff, uint8_t* data, uint16_t* size);  //  环形队列缓存的实际最大长度应该小于等于 kParserBufferLength
    uint16_t create_frame( uint8_t* frame, uint8_t* data,uint16_t data_size);

    static uint16_t pack_data(uint8_t* frame, uint8_t* data,uint16_t data_size, uint8_t frame_id);
    static Simple_Frame_Parser::ParseState unpack_data(uint8_t* frame, uint8_t* data ,uint16_t* data_size,uint8_t frame_id);
}Simple_Frame;


}
