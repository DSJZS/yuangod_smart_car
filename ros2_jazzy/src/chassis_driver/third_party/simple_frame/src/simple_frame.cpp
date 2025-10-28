#include "simple_frame/simple_frame.hpp"

namespace cya::hal::protocol
{

Simple_Frame_Parser::Simple_Frame_Parser( uint8_t packet_head )
    : packet_head_( packet_head )
{}

void Simple_Frame_Parser::set_packet_head( uint8_t packet_head )
{
    this->packet_head_ = packet_head;
}

bool Simple_Frame_Parser::pack_data(uint8_t* packet, uint8_t* data,
        uint16_t data_size,uint8_t packet_id)
{
    uint8_t sum = 0;

    packet[0] = packet_id;
    sum += packet[0];

    if( data_size + 3 > 256 )
        return false;

    packet[1] = data_size + 3;
    
    sum += packet[1];

    for( int i = 2 ; i < packet[1] - 1 ; ++i)
    {
        packet[i] = data[ i - 2 ];
        sum += packet[i];
    }

    packet[ packet[1] - 1 ] = sum;

    return true;
}

Simple_Frame_Parser::ParseState Simple_Frame_Parser::unpack_data(uint8_t* packet, uint8_t* data ,uint16_t* size,uint8_t packet_id)
{
    if( packet_id != packet[0] )
        return Simple_Frame_Parser::ParseState::HEAD_BYTE_ERR;

    uint16_t packet_size = packet[1];

    if( packet_size < Simple_Frame_Parser::kCommandMinLength )
        return Simple_Frame_Parser::ParseState::SIZE_BYTE_ERR;
    
    uint8_t sum = 0;
    sum += packet[0];
    sum += packet[1];
    for( int i = 2 ; i < ( packet_size - 1 ) ; ++i )
    {
        data[i-2] = packet[i];
        sum += packet[i];
    }
    if( sum != packet[packet_size - 1] )
        return Simple_Frame_Parser::ParseState::CHECK_SUM_ERR;

    if( size )
        *size = packet[1] - 3;

    return Simple_Frame_Parser::ParseState::NO_ERR;
}

bool Simple_Frame_Parser::get_command( lwrb_t* buff, uint8_t* data, uint16_t* size)
{
    Simple_Frame_Parser::ParseState state = Simple_Frame_Parser::ParseState::NO_ERR;
    uint16_t packet_size = 0;
    uint16_t skip_size = 0;

    uint8_t packet[Simple_Frame_Parser::kParserBufferLength] = {0}; //  以后改为动态数组

    packet_size = lwrb_peek( buff, 0, packet, Simple_Frame_Parser::kParserBufferLength );

    if( packet_size < Simple_Frame_Parser::kCommandMinLength )
        return false;

    while( 1 )
    {
        state = Simple_Frame_Parser::unpack_data( packet + skip_size, data, size,this->packet_head_ );

        if( state == Simple_Frame_Parser::ParseState::HEAD_BYTE_ERR ||
            state == Simple_Frame_Parser::ParseState::SIZE_BYTE_ERR ||
            state == Simple_Frame_Parser::ParseState::CHECK_SUM_ERR   )
        {
            ++skip_size;
            if( ( packet_size - skip_size ) < Simple_Frame_Parser::kCommandMinLength )
            {
                lwrb_skip( buff, skip_size);
                return false;
            } else {
                continue;
            }
        } else if( state == Simple_Frame_Parser::ParseState::NO_ERR ){
            lwrb_skip( buff,  skip_size + packet[skip_size+1] );
            return true;
        }
    }
}

}
