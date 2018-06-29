#ifndef SYSTEM_H
#define SYSTEM_H

#define PRODUCT_ID (2)
#define SYSTEM_TYPE_ID (1)
#define DEVELOP_VERSION_CODE (7)
#define OFFICIAL_VERSION_CODE (7)

#define AUTO_MODE  (1)
#define MANUAL_MODE (0)

extern system_t g_system;
extern env_t g_env;
extern motion_t g_motion;
extern int cmd_hs;

#define SET_CONTROLLER_MODE(data)  (g_system.auto_enable = data)
#define SET_MANUAL_WORK_MODE(data)  (g_system.manual_work_mode = data)
#define SET_AUTO_WORK_MODE(data)  (g_system.auto_work_mode = data)
#define SET_MIN_Z(data) (g_system.min_z = data)
#define SET_MAX_Z(data) (g_system.max_z = data)
#define SET_AUTO_MAX_VX(data) (g_system.max_vx = data)
#define SET_AUTO_MIN_VX(data) (g_system.min_vx = data)
#define SET_AUTO_MAX_VTH(data) (g_system.max_vth = data)

extern int cmp_equal(int a,int b);
extern int cmp_inequal(int a,int b);
extern int  set_upper_server_ip(unsigned int ip);
extern int upgrade_replace_nav_file(void);
extern int set_base_high_limit(double limit);
extern int set_base_low_limit(double limit);

extern int send_pkg(unsigned short int pkg_type,int data,int type,unsigned char * str);
extern int handle_led_power(system_t *sys);
extern void handle_vel(system_t *sys);
extern void handle_handspike(system_t *sys);
extern void handle_sensors_info(system_t *sys);
extern int read_files(system_t *sys,env_t *env);
extern void init_system_param(system_t *sys,motion_t *motion,env_t *env);
extern void handle_system_params(system_t *sys,motion_t *motion,env_t *env);
extern void handle_system_status(system_t *sys,motion_t *motion,env_t *env);
extern int set_sensors_cmd(system_t *sys);

extern void handle_movebase(system_t *sys,motion_t *motion);
extern void init_sys_thread(system_t *sys);

extern void get_system_version_code(system_t *sys);


inline void set_int_buf(unsigned char *buf,int data)
{
    buf[0] = *((unsigned char *)&data);
    buf[1] = *((unsigned char *)&data+1);
    buf[2] = *((unsigned char *)&data+2);
    buf[3] = *((unsigned char *)&data+3);
}

#endif

