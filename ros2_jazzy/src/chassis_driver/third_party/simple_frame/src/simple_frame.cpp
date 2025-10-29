#include "simple_frame/simple_frame.hpp"

namespace cya::hal::protocol
{

Simple_Frame_Parser::Simple_Frame_Parser( uint8_t frame_head )
    : frame_head_( frame_head )
{}

void Simple_Frame_Parser::set_frame_head( uint8_t frame_head )
{
    this->frame_head_ = frame_head;
}

uint16_t Simple_Frame_Parser::pack_data(uint8_t* frame, uint8_t* data,
        uint16_t data_size,uint8_t frame_id)
{
    uint8_t sum = 0;

    frame[0] = frame_id;
    sum += frame[0];

    if( data_size + 3 > Simple_Frame_Parser::kParserBufferLength )
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

Simple_Frame_Parser::ParseState Simple_Frame_Parser::unpack_data(uint8_t* frame, uint8_t* data ,uint16_t* size,uint8_t frame_id)
{
    if( frame_id != frame[0] )
        return Simple_Frame_Parser::ParseState::HEAD_BYTE_ERR;

    uint16_t frame_size = frame[1];

    if( frame_size < Simple_Frame_Parser::kCommandMinLength )
        return Simple_Frame_Parser::ParseState::SIZE_BYTE_ERR;
    
    uint8_t sum = 0;
    sum += frame[0];
    sum += frame[1];
    for( int i = 2 ; i < ( frame_size - 1 ) ; ++i )
    {
        data[i-2] = frame[i];
        sum += frame[i];
    }
    if( sum != frame[frame_size - 1] )
        return Simple_Frame_Parser::ParseState::CHECK_SUM_ERR;

    if( size )
        *size = frame[1] - 3;

    return Simple_Frame_Parser::ParseState::NO_ERR;
}

bool Simple_Frame_Parser::get_command( lwrb_t* buff, uint8_t* data, uint16_t* size)
{
    Simple_Frame_Parser::ParseState state = Simple_Frame_Parser::ParseState::NO_ERR;
    uint16_t frame_size = 0;
    uint16_t skip_size = 0;

    uint8_t frame[Simple_Frame_Parser::kParserBufferLength] = {0}; //  以后改为动态数组

    frame_size = lwrb_peek( buff, 0, frame, Simple_Frame_Parser::kParserBufferLength );

    if( frame_size < Simple_Frame_Parser::kCommandMinLength )
        return false;

    while( 1 )
    {
        state = Simple_Frame_Parser::unpack_data( frame + skip_size, data, size,this->frame_head_ );

        if( state == Simple_Frame_Parser::ParseState::HEAD_BYTE_ERR ||
            state == Simple_Frame_Parser::ParseState::SIZE_BYTE_ERR ||
            state == Simple_Frame_Parser::ParseState::CHECK_SUM_ERR   )
        {
            ++skip_size;
            if( ( frame_size - skip_size ) < Simple_Frame_Parser::kCommandMinLength )
            {
                lwrb_skip( buff, skip_size);
                return false;
            } else {
                continue;
            }
        } else if( state == Simple_Frame_Parser::ParseState::NO_ERR ){
            lwrb_skip( buff,  skip_size + frame[skip_size+1] );
            return true;
        }
    }
}

uint16_t Simple_Frame_Parser::create_frame( uint8_t* frame, uint8_t* data,uint16_t data_size)
{
    return this->pack_data( frame, data, data_size, this->frame_head_);
}

}
