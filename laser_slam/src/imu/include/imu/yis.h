#ifndef YIS_H
#define YIS_H

#include "imu/config.h"

typedef struct{
    
    int com_device;
    char dev[DEVICE_NAME_LEN];

    double ax;
    double ay;
    double az;

    double fax;
    double fay;
    double faz;

    double dvx;
    double dvy;
    double dvz;

    double wx;
    double wy;
    double wz;

    double mx;
    double my;
    double mz;

    double pitch;
    double roll;
    double yaw;

    double q0;
    double q1;
    double q2;
    double q3;

    double dq0;
    double dq1;
    double dq2;
    double dq3;

}yis_sys_t;

#endif