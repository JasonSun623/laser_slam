#ifndef LED_H
#define LED_H

#define POWER_CURRENT_LEN 33
#define LED_UPGRADE_FILE_FRAME_LEN 240
#define LED_UPGRADE_OVER_TIME 5*60
#define LED_HARDWARE_VER_LEN 2
#define LED_SOFTWARE_VER_LEN 11
#define LED_READY_UPGRADE_SLEEP_TIME 5000*1000
#define LED_END_UPGRADE_SLEEP_TIME 5000*1000
#define LED_SLEEP_TIME 60*1000
#define POWER_ERROR_DATA_LEN 2

typedef struct{
    int handle_data_flag;

    unsigned char heart_beat_flag;

    unsigned char pkg_type;
    unsigned int work_flag;
    LED_POWER_TYPE cmd_mode;           //command
    LED_POWER_TYPE fb_mode;
    LED_EFFECT_TYPE cmd_effect;
    LED_EFFECT_TYPE fb_effect;
	
    unsigned char power_status1;//power system status
    unsigned char power_status2;
    unsigned char power_v1;
	unsigned char power_v2;
    unsigned char power_p1;
    unsigned char power_p2;
	unsigned char charger_power_v1;
	unsigned char charger_power_v2;
    unsigned char power_switch_status1;
	unsigned char power_switch_status2;
	unsigned char power_switch_status3;
	unsigned char power_switch_status4;

	unsigned int get_power_flag; 
    unsigned char power_i[POWER_CURRENT_LEN*2];
    unsigned char error_passageway;
	unsigned char error_data[POWER_ERROR_DATA_LEN];
    unsigned char sys_status;   //led and power system status
    unsigned char err1;
    unsigned char err2;
	unsigned char err3;
	unsigned char err4;

	unsigned char ctrl_power_ack;
	unsigned char infrared_light;
	
    double led_freq;
    int startup_flag;
    int work_normal;
    int prior_led;

    int com_rssi;       //the interface judge com communication
    int rec_num;
    com_state_e com_state;
    int com_device;
    char dev[DEVICE_NAME_LEN];
    unsigned char hardware_version[LED_HARDWARE_VER_LEN];
    unsigned char software_version[LED_SOFTWARE_VER_LEN];
	char version[BUF_LEN];
    char version_flag;
	char name[FILE_PATH_LEN];
	char md5[MD5_SIZE];
	unsigned char upgrade_status;
	int upgrade_result;
	
    int power_current_temp_err;
	int error_power_status;
	int get_power_status;
	
}led_power_sys_t;

typedef struct{
	int recv_type;
	int recv_len;
	int current_ctrl_type;
	int current_ctrl_rlt;
	int ctrl_power_ack;
	int get_power_current_ack;
	int fan_switch_ack;
	int upgrade_type;
	int upgrade_ready_rlt;
	int upgrade_recv_rlt;
	int upgrade_end_rlt;
	int power_current_temp_err;
	unsigned char err1;
    unsigned char err2;
	unsigned char err3;
	unsigned char err4;
	unsigned char power_i_freq;
}led_info_t;

extern int led_upgrade(char * path,char * md5char);
extern void get_led_version(void);
extern int set_led_power_function(int module,int command);
extern int set_led_power_effect(LED_POWER_TYPE mode,LED_EFFECT_TYPE effect);
extern void infrared_light_ctrl(int type,int light);
extern void fan_switch_ctrl(int fan_switch);

extern led_power_sys_t *get_led_power_info(void);
extern void *led_thread_start(void *);
extern void set_led_prior(int type,int value);
extern int get_led_prior(void);
extern int set_power_upgrade(char *str,char *md5);
extern int get_power_upgrade_status(void);
extern int get_power_upgrade_result(void);

extern void set_get_power_flag(void);
extern void clear_error_power_status(void);
extern void clear_get_power_status(void);

#endif

