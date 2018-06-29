#ifndef CONFIG_H
#define CONFIG_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/MagneticField.h>

#define BUF_LEN (256)
#define DEVICE_NAME_LEN (50)
#define PI (3.1415926)

extern void set_speed(int fd, int speed);
extern int set_parity(int fd,int databits,int stopbits,int parity);
extern int open_com_device(char *dev);

#endif