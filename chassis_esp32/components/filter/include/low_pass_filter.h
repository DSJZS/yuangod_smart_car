#ifndef __LOW_PASS_FILTER__
#define __LOW_PASS_FILTER__

#ifdef __cplusplus
extern "C"
{
#endif

float low_pass_filter( float alpha, float old_data, float new_data);

#ifdef __cplusplus
}
#endif

#endif
