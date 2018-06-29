#ifndef REPORT_H
#define REPORT_H


#define BASE_LASER_ERR_LIMIT (0.254)
#define SENSOR_LASER_ERR_LIMIT (2.1)
#define SENSOR_SONAR_ERR_LIMIT (2.1)

typedef enum{
    MODULE_PAD = 1,
    MODULE_DLP,
    MODULE_NAV,
    MODULE_SENSOR,
    MODULE_LED,
    MODULE_BASE,
    MODULE_POWER,
    MODULE_CAMERA,
}module_e;

typedef enum{
    LEVEL_ERROR = 0,
    LEVEL_WARN,
    LEVEL_INFO,
    LEVEL_DEBUG,
}level_e;

typedef struct{
    module_e module_group;
    module_e module;
    unsigned char function;
}ask_status_t;

typedef struct{
    level_e level;
    module_e module;
    unsigned char function;
    unsigned char len;
    unsigned char data[SOCKET_PKG_LEN];
}ans_status_t;

typedef struct{
    ans_status_t ans;
    int flag;
}event_t;


extern int report_event(ans_status_t *ans);
extern int handle_report_status(ask_status_t ask,ans_status_t *ans,system_t *sys,
           motion_t *motion, env_t *env);
extern int set_event_buffer(ans_status_t *ans);
extern void handle_report_event(void);

extern void init_event_buf(void);



#endif
