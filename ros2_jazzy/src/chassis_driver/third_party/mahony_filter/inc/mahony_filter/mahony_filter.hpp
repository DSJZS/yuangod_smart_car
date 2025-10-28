
#ifndef __MAHONY_FILTER_H__
#define __MAHONY_FILTER_H__

typedef struct {
    double w;
    double x;
    double y;
    double z;
} Imu_Quaternion;

void mahony_filter(Imu_Quaternion* quaternion,float gx, float gy, float gz, float ax, float ay, float az);

#endif


