#ifndef CONFIG_H
#define CONFIG_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Joy.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>   
#include <unistd.h>     
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      
#include <termios.h>   
#include <errno.h>     
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netinet/in.h>



#define MAX_POINT (128)
#define PI (3.1415926)
#define LEN_CM_TO_M (0.01)
#define LEN_MM_TO_M (0.001)
#define LEN_M_TO_CM (100.0)
#define LEN_M_TO_MM (1000.0)

#define EQUAL_VALUE  (0.01)
#define MAX_MAP_VALUE (10000.0)
#define EQUAL_XY_VALUE (0.05)
#define EQUAL_TH_VALUE (0.1)
#define MD5_SIZE (16)
#define OBSTACLE_STOP_SCALE (0.0)
#define OBSTACLE_SLOW_SCALE (0.5)
#define OBSTACLE_FREE_SCALE (1.0)
#define MIN_BASE_VX (0.05)
#define MIN_BASE_VTH (0.12)
#define CAMERA_POS_X (0.163)
#define CAMERA_POS_Y (0.0)
#define CAMERA_POS_TH (0.0)


#define LASER_NUM  (13)
#define SONAR_NUM  (9)
#define HALL_NUM    (2)
#define SENSOR_STATUS_NUM (3)
#define SOCKET_PKG_LEN (2048)
#define UPPER_COM_RECV_LEN     (2048)
#define UPPER_COM_HANDLE_LEN (4096)
#define SOCKET_PKG_BUF_SIZE  (1792)
#define BUF_LEN (256)
#define FRAME_BUF_LEN (40)

#define BASE_LASER_NUM   (3)
#define BASE_MOTOR_NUM (2)
#define HANDSPIKE_STATUS_NUM (3)
#define LED_POWER_COM_PERIOD (20)
#define PUSHED_FLAG_OBS_TIMES  (5)
#define MODIFY_POS_OBS_TIMES  (9)
#define MODIFY_POS_OVER_TIMES  (200)


#define BASE_MOTOR_ERR_BIT  (0x7f)
#define BASE_SAFE_MODE_BIT  (0x01)
#define BASE_POWER_ON_BIT  (0x02)
#define BASE_STOP_BIT  (0x78)
#define BASE_SENSOR_BUMPER (0x70)
#define BASE_SENSOR_LASER (0x07)
#define BASE_STOP_ERR (0x70)
#define BASE_STOP_ESTOP (0x08)
#define BASE_ESTOP_KEY_BIT (0x10)


#define POWER_CURRENT_NUM (33)
#define POWER_VERSION_LEN (11)


#define CLOSE_WAIT_TIME (50000)
#define DEVICE_NAME_LEN (50)
#define MARK_COUNT_NUM (20)
#define SYSTEM_CFG_ITEM_NUM (47)
#define FILE_PATH_LEN  (128)
#define UPGRADE_OVER_TIME (1200)

#define MANUAL_BASIC_MASK (0x0)
#define MANUAL_MAP_MASK   (0x20)
#define AUTO_BASIC_MASK  (0x80)
#define AUTO_DANCE_MASK  (0xA0)
#define ERROR_MASK  (0x10)
#define WARN_MASK   (0x08)
#define UPGRADE_MAX_FILESIZE (20971520)
#define VERSION_LEN  (11)
#define ERROR_PASSAGEWAY_DATA_LEN (3)
#define VERSION_POINT_NUM (4)


#define PRINT_ENABLE (1)
extern FILE *fp;

typedef enum{
    COM_OPENING = 1,
    COM_CHECK_VERSION,
    COM_RUN_OK,
    COM_CLOSING,
}com_state_e;

typedef enum{
    BASE_NO_CMD = 0,
    BASE_OPEN_SAFE_MODE_CMD,
    BASE_CLOSE_SAFE_MODE_CMD,
    BASE_SET_LASER_PARAM_CMD,
    BASE_CLEAR_POWER_ON_CMD,
    BASE_CLEAR_ERR_CMD,
}base_cmd_e;


typedef enum{
    READ_SYS_FILE_ERR = 0x01,
    WRITE_SYS_FILE_ERR,
    READ_MARK_FILE_ERR,
    WRITE_MARK_FILE_ERR,
    READ_TPOINT_FILE_ERR,
    WRITE_TPOINT_FILE_ERR,
    READ_GOAL_FILE_ERR,
    WRITE_GOAL_FILE_ERR,
    READ_MAP_FILE_ERR,
    WRITE_MAP_FILE_ERR,
    
    CREATE_THREAD_ERR,
    MALLOC_FAILED_ERR,
    
    SENSOR_MODULE_ERR,
    BASE_MODULE_ERR,
    UPPER_MODULE_ERR,
    CAMERA_MODULE_ERR,
    BASE_UNNORMAL_POWER_ON_ERR,

    DANCE_ESTOP_ERR,
    NAV_PATH_ERR,
    NAV_CAL_PATH_ERR,
    NAV_CHANGE_PATH_ERR,
    NAV_FIND_PATH_ERR,
    SAVE_MARK_ERR,
}ERR_TYPE;

typedef enum{
    WARN = 1,
}WARN_TYPE;

typedef enum{
    DANCING = 0x01,
    DANCE_AFTER_WAIT,
    DANCE_BACK_START,
    DANCE_FINISHED,
    DANCE_ERROR,
}DANCE_STATE_TYPE;

typedef enum{
    CAMERA_NO_DEVICE = 0x00,
    CAMERA_LOST_CONNECTED,
    CAMERA_NORMAL_WORK,
    CAMERA_EASY_WORK,
}CAMERA_STATE_TYPE;


typedef enum{
    AUTO_BASIC_MODE = 0x00,
    AUTO_DANCE_MODE,
}AUTO_MODE_TYPE;

typedef enum{
    MANUAL_BASIC_MODE = 0,
    MANUAL_MAP_MODE,
}MANUAL_MODE_TYPE;

typedef enum{
    LINE_DANCE = 0,
    ROTATE_DANCE,
    CIRCLE_DANCE,
}DANCE_TYPE;

typedef enum{
    GOAL_FILE = 0,
    TPOINT_FILE = 1,
    MARK_FILE = 2,
    MAP_FILE = 3,
    DANCE_FILE = 4,
    DANCE_INDEX_FILE = 5,
    LED_FILE = 6,

    SYSTEM_FILE = 100,
    CAMERA_FILE = 101,
    BASE_FILE = 102,
    SENSOR_FILE = 103,
    LED_POWER_FILE = 104,
}FILE_TYPE;

typedef enum{
    LED_POWER_NO_CMD = 0,
    LED_POWER_SET_WELCOME,
    LED_POWER_SET_GOODBYE,
    LED_POWER_SET_IDLE,
    LED_POWER_SET_DANCE,
    LED_POWER_SET_WARN,
    LED_POWER_SET_LOST_CONNECT,
    LED_POWER_SET_STOP,
    LED_POWER_SET_HELLO,
    LED_POWER_FREEDOM,
}LED_POWER_TYPE;


typedef enum{
    LED_DEFAULT = 1,
    LED_NORMAL,
    LED_EASY_DANCE,
    LED_NAV,
    LED_RED_WARN,
    LED_YELLOE_WARN,
    LED_WATER_GREEN,
    LED_BREATH_COLORFUL,
    LED_BREATH_PURPLE,
    LED_RED_METEOR,
    LED_YELLOW_METEOR,
    LED_GREEN_METEOR,
    LED_RED_LONG,
    LED_GREEN_LONG,
    LED_BLUE_LONG,

    LED_DANCE_DEFAULT=0xff00,
    LED_DANCE_DAWANG,
    LED_DANCE_DUJUAN,
}LED_EFFECT_TYPE;


//for sensor board
typedef struct{
    double laser_len[LASER_NUM];
    double sonar_len[SONAR_NUM];
    double estop_limit;                      //sensor emergency stop io limit
    double estop_fb_limit;                 //sensor emergency stop io actual limit on board
    double slow_limit;  
    double sonar_event1_limit;
    
	
    int stop_flag;
    int slow_flag;
    unsigned char estop_io_flag;      //sensor board emergency  stop io output;
    unsigned char infrared_flag;
    unsigned char status[SENSOR_STATUS_NUM];
    int work_normal;
    int com_rssi;
    unsigned char version[VERSION_LEN];
    unsigned char download_index;
    unsigned char upgrade_status;
    unsigned char upgrade_md5[MD5_SIZE];
    unsigned char upgrade_result;
}sensor_t;

typedef struct{
    double x;
    double y;
    double z;
    double th;
}point_t;

typedef struct{
    double vx;
    double vy;
    double vth;
}vel_t;

typedef struct{
    double x;
    double y;
    double th;
}transform_t;

typedef struct{
    point_t point;
    point_t assemble_pos;
    int id;
    CAMERA_STATE_TYPE state;
    int watch_time;                  //to watch star_info msg in n cycle;
    int rcv_num;   //count received star_info msg num;
    int get_mark_flag;
    int lose_mark_flag;
}sys_camera_t;

typedef struct{
    unsigned char state;
    unsigned char cmd;
    unsigned char mode;
    LED_EFFECT_TYPE effect;
    LED_EFFECT_TYPE act_effect;
}sys_led_t;


typedef struct{
    point_t odom;
    vel_t fb_vel;
    vel_t fb_cmd_vel;
    double laser[BASE_LASER_NUM];
    unsigned char move_status;	  //movebase controller status
    unsigned char move_rssi; 	 //movebase controller judge com communication
    unsigned char com_rssi;        //motion controller judge com communication
    unsigned char com_state;
    unsigned char estop_sensor_flag;//all of base sensor estop flag
    unsigned char motor_status[BASE_MOTOR_NUM];   //motor status
    unsigned char stop_status;    //if base error,should stop 
    unsigned char power_on_flag;
    unsigned char power_v;
    unsigned char move_backup;
    unsigned char version[VERSION_LEN];
    unsigned char download_index;
    unsigned char upgrade_status;
    unsigned char upgrade_md5[MD5_SIZE];
    unsigned char upgrade_result;
	unsigned char move_sensor_state;

    double laser_high_limit;
    double laser_low_limit;
    base_cmd_e cmd;
    int work_normal;
}sys_base_t;

typedef struct{
    point_t s_point;
    point_t e_point;
    double entire_s;   //move s
    double entire_t;   //time
    double radius;   //if circle,circle radius
    double current_s;
    double current_t;
    vel_t max_vel;
    vel_t move_vel;
    double time;
    double acc_vx;
    double acc_vth;
    DANCE_TYPE type;   //move type,circle or line
}dance_move_t;

typedef struct{
	dance_move_t *dance_move;
	vel_t max_vel;
	vel_t last_vel;
	double max_acc_x;
	double max_acc_th;
	double time;
	double start_time;
	int dance_move_num;
	int dance_move_index;
}dance_seq_t;

typedef struct{
	int set_led_effect_flag;
	long int act_id;
}led_seq_t;

typedef struct{
	int cfg_file_size;
	int cfg_file_rcv_size;
	char cfg_file_name[32];
	char *cfg_file;
	unsigned char cfg_file_crc;
	
	int index_file_size;
	int index_file_rcv_size;
	char index_file_name[32];
	char *index_file;
	unsigned char index_file_crc;

	int led_file_size;
	int led_file_rcv_size;
	char led_file_name[32];
	char *led_file;
	unsigned char led_file_crc;

	double dance_xy_range;         //dance start point tolerance in x\y 
    double dance_th_range;         //dance start point tolerance in th
    double dance_xy_scale;

	
    point_t dance_start_point;    //dance start point
    char dance_start_point_set;    //dance start point setup finish or not
    char dance_rotate_flag;
	char dance_info_flag;
    DANCE_STATE_TYPE dance_state;  //dance state

    int dance_num;                 //the num dance 
    int dance_time;                //dance current time
    int dance_entire_time;         //this dance entire time
    dance_seq_t dance_seq;
	led_seq_t led_seq;
	long int dance_id;
}dance_t;

typedef struct{	
    int led_sys_status;     //led status report to upper
	
    unsigned char cmd;           //command
    unsigned char act_mode;
    LED_EFFECT_TYPE effect;
    LED_EFFECT_TYPE act_effect;
	
    unsigned char power_status1;//power system status
    unsigned char power_status2;
    unsigned char power_v1;
	unsigned char power_v2;
    unsigned char power_p1;
    unsigned char power_p2;
    unsigned char power_switch_status1;
	unsigned char power_switch_status2;
	unsigned char power_switch_status3;
	unsigned char power_switch_status4;

    unsigned char power_i[POWER_CURRENT_NUM*2];

    unsigned char sys_status;   //led and power system status
    unsigned char err1;
    unsigned char err2;
	unsigned char err3;
	unsigned char err4;

    unsigned char version[VERSION_LEN];
    unsigned char download_index;
    unsigned char upgrade_status;
    unsigned char upgrade_md5[MD5_SIZE];
    unsigned char upgrade_result;

	int error_power_status;
	int get_power_status;
	unsigned char error_power_passageway_data[ERROR_PASSAGEWAY_DATA_LEN];
	
    int work_normal;
    int com_rssi;       //the interface judge com communication
    com_state_e com_state;	
	int mmi_test_flag;
}led_power_t;

typedef struct{
    FILE_TYPE type;
    int upgrade_file_size;
    int upgrade_file_rcv_size;
    char upgrade_file_name[32];
    char *upgrade_file;
    unsigned char upgrade_file_crc;
}upgrade_t;

typedef struct{
    sensor_t sensor;
    point_t video_to_center;
    sys_camera_t camera;
    sys_base_t base;
    dance_t dance;
    led_power_t led_power;
    upgrade_t upgrade;
	
    double max_vx;
    double min_vx;
    double max_vth;
    double obstacle_scale;         //avoid obstacle in auto mode
    double tolerance_pass;
    double tolerance_goal;
	
    vel_t tele_vel;
    vel_t plan_vel;
    vel_t real_vel;
    vel_t last_vel;
    
    int handspike;

    double min_z;
    double max_z;
    double max_accx;
    double max_accth;
    double control_freq;
    double observe_dist;
    double local_map_offset;
    double local_map_len;
    double max_manual_vx;
    double max_manual_vth;
    double rotate_limit;
    double rotate_pid_p;
    double line_vx_pid_p;
    double line_vx_pid_i;
    double line_vth_dis_p;
    double line_vth_ang_p;
    double final_rotate_pid_p;
	double nav_over_time;
	double manual_control_over_time;
	double enter_event_limit;
	double goal_nearby_range;
	double goal_nearby_th;
	double reserve_double_3;
	double reserve_double_4;
	

    int mark_num;
    int goal_num;
    int goal_count;
    int tpoint_num;
    int tpoint_count;
    int found_mark;
    int observe_times;
    int upper_work_normal;
    int print_enable;
	int enter_event_type;
	int enter_sonar_num;
	int reserve_int_1;
	int reserve_int_2;
	 
    ERR_TYPE err_num;
    WARN_TYPE warn_num;
    int sys_status;                     //

    AUTO_MODE_TYPE auto_work_mode;
    MANUAL_MODE_TYPE manual_work_mode;
    
    char auto_enable;
    char pause;
    
    char build_cord_flag;
    char set_mark_flag;
    char mark_count;
    char read_system_file_flag;
	char pushed_flag;
	char need_modify_pos_flag;
	char event_stop_flag;
	char manual_stop_overtime_flag;
	char manual_overtime_flag;
	char read_map_files_flag;       //to map
	char need_update_map_flag;

    
    char stop_plan_path;
    char change_path_flag;
	char starline_download_index;
	char starline_upgrade_status;
	char system_upgrade_flag;
	unsigned char starline_md5[MD5_SIZE];
	unsigned char system_version[VERSION_LEN];
    char navigation_download_index;
	char navigation_upgrade_status;
	unsigned char navigation_md5[MD5_SIZE];
    
    char tmp_flag;

    int pub_base_tf_;
    ros::Publisher odom_pub;
}system_t;

typedef struct{
    point_t point;
    int id;
    int cal_flag;
}mark_t;

typedef struct{
    point_t point;
    int id;
    int linked_p;
    char name[8];
    char set_flag;
}goal_t;

typedef struct{
    point_t point;
    int id;
    char set_flag;
}tpoint_t;

typedef struct{
    point_t *path_point;
    int current_index;
    int start_index;
    int end_index;
    int goal_id;
    char path_finish;
    char goal_rotate_flag;
    char started;
    char info_flag;
}path_t;

typedef struct{
    int index;
    int linked_index;
    double value;
    int min_value_flag;
}vpmap_t;

//for environment of the place
typedef struct{
    mark_t *env_mark;
    goal_t *env_goal;
    tpoint_t *env_tpoint;
    double *env_map;
    char *local_map;
    char *goal_file;
    char *tpoint_file;
    char *mark_file;
    char *map_file;
    int goal_file_size;
    int goal_file_rcv_size;
    int tpoint_file_size;
    int tpoint_file_rcv_size;
    int mark_file_size;
    int mark_file_rcv_size;
    int map_file_size;
    int map_file_rcv_size;
}env_t;

//for motion information
typedef struct{
    path_t path;
    transform_t tran;
    point_t odom;
    point_t current;
}motion_t;

typedef enum{
    SYS_ERR = 0x01,
    SYS_WARN = 0x02,
}SYS_TYPE;

typedef enum{
    GOOD_LEVEL = 3,
    MIDDLE_LEVEL = 2,
    BAD_LEVEL = 1,
    NO_CONNECT_LEVEL = 0,
}SOCKET_RSSI_LEVEL;

typedef enum{
    BUILD_SOCKET = 1,
    CFG_SOCKET,
    BIND_SOCKET,
    LISTEN_SOCKET,
    ACCEPT_SOCKET,
    NORMAL_SOCKET,
}SOCKET_STATE;


typedef struct{
    int handle_upper_com_flag;

    int need_read_flag;
    int read_num;
    unsigned char upper_com_buf[UPPER_COM_HANDLE_LEN];


    com_state_e com_state;
    int com_device;
    char dev[DEVICE_NAME_LEN];
    int com_rssi;
    int rec_num;
    
    double upper_com_freq;

    int work_normal;
    int upper_beat_flag;

    unsigned int server_ip;
    int socket_error;
    int socket_status;
    int srv_socket;
    int client_socket;
    struct sockaddr_in srv_addr;
    struct sockaddr_in client_addr;

}upper_com_sys_t;

extern void set_speed(int fd, int speed);
extern int set_parity(int fd,int databits,int stopbits,int parity);
extern int open_com_device(char *dev);
extern int read_system_file(system_t *sys);
extern int write_system_file(system_t *sys);
extern int read_led_file(system_t *sys,char *buf);
extern int handle_upgrade_file_begin(system_t *sys, unsigned char *buf);
extern int handle_upgrade_file_data(system_t *sys, unsigned char *buf);
extern int handle_upgrade_file_done(system_t *sys, unsigned char *buf);
extern int check_upgrade_system(system_t *sys,env_t *env,unsigned char *data,int *force);
extern upper_com_sys_t* get_upper_com_system_info(void);
extern int send_status_back(unsigned char *buf,int num);

//upper_com.cpp
extern void set_upper_beat_flag(int data);
extern int upper_socket_status(void);
extern void *upper_com_thread_start(void *);
extern unsigned int get_upper_server_ip(void);

//cloud.cpp
extern void *cloud_com_thread_start(void *);

#endif
