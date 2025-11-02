#include "simple_frame/simple_frame.h"

//  数据包最小长度
#define kCommandMinLength   ( (uint16_t)4 )
//  数据包解析缓存区最大长度
#define kParserBufferLength ( (uint16_t)512 )

void sfp_init( Simple_Frame* sfp_, uint8_t frame_head)
{
    sfp_->frame_head_ = frame_head;
}

void sfp_set_frame_head( Simple_Frame* sfp_, uint8_t frame_head)
{
    sfp_->frame_head_ = frame_head;
}

uint16_t sfp_pack_data( uint8_t* frame, uint8_t* data,uint16_t data_size, uint8_t frame_id)
{
    uint8_t sum = 0;

    frame[0] = frame_id;
    sum += frame[0];

    if( data_size + 3 >  kParserBufferLength )
        return 0;

    frame[1] = data_size + 3;
    
    sum += frame[1];

    for( int i = 2 ; i < frame[1] - 1 ; ++i)
    {
        frame[i] = data[ i - 2 ];
        sum += frame[i];
    }

    frame[ frame[1] - 1 ] = sum;

    return frame[1];
}

ParseState sfp_unpack_data( uint8_t* frame, uint8_t* data ,uint16_t* size,uint8_t frame_id)
{
    if( frame_id != frame[0] )
        return HEAD_BYTE_ERR;

    uint16_t frame_size = frame[1];

    if( frame_size <  kCommandMinLength )
        return SIZE_BYTE_ERR;
    
    uint8_t sum = 0;
    sum += frame[0];
    sum += frame[1];
    for( int i = 2 ; i < ( frame_size - 1 ) ; ++i )
    {
        data[i-2] = frame[i];
        sum += frame[i];
    }
    if( sum != frame[frame_size - 1] )
        return CHECK_SUM_ERR;

    if( size )
        *size = frame[1] - 3;

    return NO_ERR;
}

uint8_t sfp_get_command( Simple_Frame* sfp_, lwrb_t* buff, uint8_t* data, uint16_t* size)
{
    ParseState state = NO_ERR;
    uint16_t frame_size = 0;
    uint16_t skip_size = 0;

    uint8_t frame[ kParserBufferLength] = {0}; //  以后改为动态数组

    frame_size = lwrb_peek( buff, 0, frame,  kParserBufferLength );

    if( frame_size <  kCommandMinLength )
        return 0;

    while( 1 )
    {
        state = sfp_unpack_data( frame + skip_size, data, size,sfp_->frame_head_ );

        if( state == HEAD_BYTE_ERR ||
            state == SIZE_BYTE_ERR ||
            state == CHECK_SUM_ERR   )
        {
            ++skip_size;
            if( ( frame_size - skip_size ) <  kCommandMinLength )
            {
                lwrb_skip( buff, skip_size);
                return 0;
            } else {
                continue;
            }
        } else if( state == NO_ERR ){
            lwrb_skip( buff,  skip_size + frame[skip_size+1] );
            return 1;
        }
    }
}

uint16_t sfp_create_frame( Simple_Frame* sfp_, uint8_t* frame, uint8_t* data,uint16_t data_size)
{
    return sfp_pack_data( frame, data, data_size, sfp_->frame_head_);
}

