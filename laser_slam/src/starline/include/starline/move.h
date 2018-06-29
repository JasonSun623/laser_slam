#ifndef MOVE_H
#define MOVE_H

#define MOVE_UPGRADE_FILE_FRAME_LEN 240
#define MOVE_UPGRADE_OVER_TIME 5*60
#define MOVE_HARDWARE_VER_LEN 4
#define MOVE_SOFTWARE_VER_LEN 11
#define MOVE_READY_UPGRADE_SLEEP_TIME 3000*1000
#define MOVE_END_UPGRADE_SLEEP_TIME  5000*1000
#define MOVE_SLEEP_TIME 60*1000

typedef struct{
    point_t odom;
    vel_t fb_vel;
    
    unsigned char move_status;    //movebase controller status
    unsigned char move_rssi;      //movebase controller judge com communication
    unsigned char estop_sensor_flag;
    unsigned char err;//to delete
    unsigned char motor_status[BASE_MOTOR_NUM];  //0->left,1->right
    unsigned char handspike_status[BASE_MOTOR_NUM];   
    unsigned char power_v;
	unsigned char move_backup;
    double move_freq;
    double laser[BASE_LASER_NUM];
    double high_limit;
    double low_limit;
    vel_t cmd_vel;
    unsigned char cmd;
    unsigned char move_sensor_state;
	int move_open_station;
    int handle_data_flag;
    int work_normal;
    int com_rssi;       //the interface judge com communication
    int rec_num;
    com_state_e com_state;
    int com_device;
    char dev[DEVICE_NAME_LEN];
	unsigned char hardware_version[MOVE_HARDWARE_VER_LEN];
	unsigned char software_version[MOVE_SOFTWARE_VER_LEN];
	char name[FILE_PATH_LEN];
	char md5[MD5_SIZE];
	unsigned char upgrade_status;
	int upgrade_result;
}move_sys_t;

typedef struct{
    int recv_type;
	int recv_len;
	
	unsigned char move_open_station_ack;
	double high_limit_ack;
	double low_limit_ack;
	unsigned char sensor_state_ack;
	unsigned char motor_status_ack[BASE_MOTOR_NUM];
	
	int upgrade_type;
	int upgrade_ready_rlt;
	int upgrade_recv_rlt;
	int upgrade_end_rlt;
}move_info_t;

extern void set_movebase_cmd_vel(vel_t vel);
extern move_sys_t *get_movebase_info(void);
extern void *movebase_thread_start(void *);

extern int clear_open_signal(void);
extern void get_open_sigal(void);
extern int set_base_limit(double high_limit,double low_limit);
extern void get_base_limit(void);
extern int set_sensor_function(unsigned char senor_state);
extern void get_sensor_function(void);
extern int clear_error_state(void);
extern void get_error_state(void);
extern void handspike_lift_send_frame(void);
extern void handspike_down_send_frame(void);
extern void handspike_stop_send_frame(void);
extern void handspike_power_send_frame(void);
extern void get_move_version(void);
extern int move_upgrade(char * path,char * md5char);
extern int set_movebase_upgrade(char *str,char *md5);
extern int get_movebase_upgrade_status(void);
extern int get_movebase_upgrade_result(void);

extern unsigned char baseStateData[];
extern unsigned char loadFlag ;
extern unsigned char loadCMD ;
//20170815,Zero
//void loadMotorCMD(uint8_t cmd);

#endif
