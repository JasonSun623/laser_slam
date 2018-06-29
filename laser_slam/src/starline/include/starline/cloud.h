#ifndef CLOUD_H
#define CLOUD_H

#include "../include/starline/md5.h"


#define NAME_LEN  (32)
#define WEB_LEN   (256)
#define FORCE_UPDATE_LEN  (8)
#define DOWNLOAD_BUF_SIZE  (10)


typedef struct{
    unsigned char name[NAME_LEN];
	unsigned char md5[MD5_STRING_LEN+1];
	unsigned char web[WEB_LEN];
	unsigned char force_update[FORCE_UPDATE_LEN];
	unsigned char version[VERSION_LEN];
}module_upgrade_t;

typedef enum{
    FINISHED = 0,
    WAIT_DOWNLOAD,
    DOWNLOADING,
    DOWNLOAD_SUCCESS,
    DOWNLOAD_FAILED,
}download_status_e;


typedef struct{
    download_status_e download_status;
	module_upgrade_t module;
	int result_flag;
}download_t;

typedef struct{
	int cloud_flag;
	download_t download_buf[DOWNLOAD_BUF_SIZE];
}cloud_t;



extern int set_avalible_cloud_buf(module_upgrade_t *upgrade);
extern download_t *get_download_buf(int i);

#endif 
