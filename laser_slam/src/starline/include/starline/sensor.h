#ifndef SENSOR_H
#define SENSOR_H

#define SENSOR_UPGRADE_FILE_FRAME_LEN 240
#define SENSOR_UPGRADE_OVER_TIME 5*60
#define SENSOR_HARDWARE_VER_LEN 4
#define SENSOR_SOFTWARE_VER_LEN 11
#define SENSOR_NUM 10
#define SENSOR_READY_UPGRADE_SLEEP_TIME 600*1000
#define SENSOR_END_UPGRADE_SLEEP_TIME 600*1000
#define SENSOR_SLEEP_TIME 60*1000

#include "starline/SensorMsg.h"
#include <sensor_msgs/PointCloud2.h>
using namespace starline;

enum{
	INFRARED = 0,
	ULTRASOUND = 1,
    HALL = 2,
};

typedef struct{
    int handle_data_flag;
    double estop_limit;
    double estop_fb_limit;
    double laser_len[LASER_NUM];
    double sonar_len[SONAR_NUM];
    bool hall_state[HALL_NUM];
    unsigned char estop_io_flag;
    unsigned char infrared_flag;
    unsigned char status[SENSOR_STATUS_NUM];
    
    double sensor_freq;
    int obstacle_flag;
    int work_normal;
    
    int com_rssi;
    com_state_e com_state;
    int rec_num;
    int com_device;
    char dev[DEVICE_NAME_LEN];
    unsigned char hardware_version[SENSOR_HARDWARE_VER_LEN];
    unsigned char software_version[SENSOR_SOFTWARE_VER_LEN];
    char name[FILE_PATH_LEN];
    char md5[MD5_SIZE];
    unsigned char upgrade_status;
	int upgrade_result;

	SensorMsg laser_data;
	SensorMsg sonar_data;
	SensorMsg hall_data;
    ros::Publisher lasercloud_pub;
	ros::Publisher sensor_pub;
}sensor_sys_t;

typedef struct{
    int recv_type;
	int recv_len;
	int set_safe_distance_ack;
	unsigned char safe_distance_set[SENSOR_NUM];
	int function_cali_cmd_rlt;
	int function_cali_param_rlt;
	int set_function_cali_ack;
	
	int upgrade_type;
	int upgrade_ready_rlt;
	int upgrade_recv_rlt;
	int upgrade_end_rlt;
}sensor_info_t;

extern void *sensor_thread_start(void *);
extern int get_sensor_data(system_t *sys);

extern void set_safe_distance(double safe_distance);
extern void get_safe_distance(void);
extern void set_function_cali(int function_cali_cmd, int function_cali_param);
extern void get_sensor_version(void);
extern int sensor_upgrade(char * path,char * md5char);
extern int set_sensor_upgrade(char *str,char *md5);
extern int get_sensor_upgrade_status(void);
extern int get_sensor_upgrade_result(void);

#endif
