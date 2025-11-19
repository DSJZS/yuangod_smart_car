#include "low_pass_filter.h"


float low_pass_filter( float alpha, float old_data, float new_data)
{
    return alpha * new_data + ( 1 - alpha ) * old_data;
}
