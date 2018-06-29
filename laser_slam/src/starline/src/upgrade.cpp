#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Joy.h"

#include <sstream>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <pthread.h>
#include "../include/starline/Id.h"
#include "../include/starline/config.h"
#include "../include/starline/system.h"
#include "../include/starline/md5.h"
#include "../include/starline/cloud.h"
#include "../include/starline/cJSON.h"
#include "../include/starline/report.h"

int handle_upgrade_file_begin(system_t *sys, unsigned char *buf)
{
	//int i = 5;
	int filesize =0;
	int file_name_len = 0;
	FILE_TYPE type;

	if((NULL == sys) ||(NULL == buf))
	{
		ROS_DEBUG("%s: sys or buf is NULL.", __func__);
		return -1;
	}

	type = (FILE_TYPE)buf[5];
	filesize = ((int)buf[9] << 24) | ((int)buf[8] << 16) | ((int)buf[7] << 8) | (int)buf[6];
	ROS_DEBUG("filesize %x",filesize);

    if((filesize <= 0) || (filesize > UPGRADE_MAX_FILESIZE))
    {
        ROS_DEBUG("upgrade file size is wrong");
        return -1;
    }
	//file_name_len = frame_len - frame_head_len - cmd_type_len - file_type - file_size_len - crc_len - frame_end_len
	file_name_len = buf[1] - 12;
	if((file_name_len <=0) || (file_name_len > ((int)sizeof(sys->upgrade.upgrade_file_name))))
	{
	    ROS_DEBUG("file name length is wrong");
	    return -1;
	}

	if(type == SYSTEM_FILE)
	{
	    sys->upgrade.type = SYSTEM_FILE;
	    sys->upgrade.upgrade_file_size = filesize;
		sys->upgrade.upgrade_file_rcv_size = 0;
		memcpy(sys->upgrade.upgrade_file_name,&buf[10],file_name_len);
		if(NULL != sys->upgrade.upgrade_file)
		{
		    free(sys->upgrade.upgrade_file);
			sys->upgrade.upgrade_file = NULL;
		}
		sys->upgrade.upgrade_file = (char *)malloc(sys->upgrade.upgrade_file_size + 1);
		ROS_DEBUG("upgrade file size :%d,%d",sys->upgrade.upgrade_file_size,filesize);
		if(NULL == sys->upgrade.upgrade_file)
		{
			ROS_DEBUG("%s: malloc cfg_file failed",__func__);
			return -1;
		}
		memset(sys->upgrade.upgrade_file, 0, sys->upgrade.upgrade_file_size + 1);
	}
	else
	{
		ROS_DEBUG("%s: undefined dance file type:%d", __func__, type);
		return -1;
	}
	return 0;
}

int handle_upgrade_file_data(system_t *sys, unsigned char *buf)
{
	//int i = 0;
	int data_len = 0;
	FILE_TYPE type;

	if((NULL == sys) || (NULL == buf))
	{
	    ROS_DEBUG("sys or buf is NULL");
		return -1;
	}

    if(sys->upgrade.upgrade_file_size <= 0)
    {
        return -1;
    }
	type = (FILE_TYPE)buf[5];
	
	if(type == SYSTEM_FILE)
	{
		if(sys->upgrade.upgrade_file_rcv_size + data_len 
			<= sys->upgrade.upgrade_file_size)
		{
			//data_len = frame_len - frame_head_len - cmd_type_len - file_type - crc_len - frame_end_len
			data_len = buf[1] - 8;
			if(data_len <= 0)
			{
			    ROS_DEBUG("data length is wrong");
			    return -1;
			}
			//ROS_DEBUG("%s: data_len %d ", __func__, data_len);

			//ROS_DEBUG("received file size:%d",sys->upgrade.upgrade_file_rcv_size);
		    memcpy(sys->upgrade.upgrade_file+sys->upgrade.upgrade_file_rcv_size,
		                 &buf[6],data_len);
			sys->upgrade.upgrade_file_rcv_size += data_len;
		}
		else
		{
			ROS_DEBUG("%s: receive cfg_file overflowed.", __func__);
			return -1;
		}

	}
	else
	{
		ROS_DEBUG("%s: undefined dance file type:%d", __func__, type);
		return -1;
	}
	return 0;
}

int handle_upgrade_file_done(system_t *sys, unsigned char *buf)
{
	int i = 0;
	FILE_TYPE type;
	unsigned char md5_upper_value[MD5_SIZE] = {0,};
	unsigned char md5_cal_value[MD5_SIZE] = {0,};

	if((NULL == sys) || (NULL == buf))
	{
	    ROS_DEBUG("sys or buf is NULL");
		return -1;
	}

    if((NULL == sys->upgrade.upgrade_file) ||
		    (sys->upgrade.upgrade_file_size <= 0))
    {
        ROS_DEBUG("upgrade file is wrong");
        return -1;
    }

	type = (FILE_TYPE)buf[5];
	memset(md5_upper_value,0,sizeof(md5_upper_value));
	i = md5_string_to_hex(&(buf[6]),md5_upper_value);
	if(0 != i)
	{
	    ROS_DEBUG("md5 string to hex error");
	    return -1;
	}

	if(type == SYSTEM_FILE)
	{
		/* cfg_file crc check */
		ROS_DEBUG("rcv size:%d,file size:%d",sys->upgrade.upgrade_file_rcv_size,
		    sys->upgrade.upgrade_file_size);
		if(sys->upgrade.upgrade_file_rcv_size == sys->upgrade.upgrade_file_size)
		{
			ROS_DEBUG("%s: rec_size=%d, upgrade_file=%d ", __func__, 
			  sys->upgrade.upgrade_file_rcv_size,sys->upgrade.upgrade_file_size);
			
			i = compute_md5((unsigned char*)(sys->upgrade.upgrade_file),
				sys->upgrade.upgrade_file_size,md5_cal_value);
			if(0 != i)
			{
			    ROS_DEBUG("compute upgrade file md5 value failed!");
				free(sys->upgrade.upgrade_file);
			    sys->upgrade.upgrade_file = NULL;
			    return -1;
			}
			for(i=0; i<MD5_SIZE; i++)
			{
				if(md5_upper_value[i] != md5_cal_value[i])
				{
				    ROS_DEBUG("upgrade file md5 check failed!");
					free(sys->upgrade.upgrade_file);
			    	sys->upgrade.upgrade_file = NULL;
				    return -1;
				}
			}
		}
		else
		{
			free(sys->upgrade.upgrade_file);
			sys->upgrade.upgrade_file = NULL;
			ROS_DEBUG("%s: upgrade_file received failed.", __func__);
			return -1;
		}
	}
	else
	{
		ROS_DEBUG("%s: undefined upgrade_file type:%d", __func__, type);
		return -1;
	}

	/* write to file */
	//i = write_upgrade_file(sys,type);
	if(0 != i)
	{
	    return -1;
	}
	free(sys->upgrade.upgrade_file);
    sys->upgrade.upgrade_file = NULL;
	return 0;
}

int read_upgrade_json_info(module_upgrade_t *upgrade,unsigned char *data)
{
    cJSON * json_data = NULL;
	cJSON * str = NULL;

	if((NULL == upgrade) || (NULL == data))
	{
	    return -1;
	}
    //ROS_DEBUG("upgrade info:%s",data);

    json_data = cJSON_Parse((char *)data);
    if(NULL == json_data)                                                                                         
    {
        // parse faild, return
        return -1;
    }
    
    //read name item
	str = cJSON_GetObjectItem(json_data, "name");
    if(NULL == str)
    {
        //get object named "hello" faild
        return -1;
    }
	if(strlen(str->valuestring) < NAME_LEN)
	{   
        memcpy(upgrade->name,str->valuestring,strlen(str->valuestring));
		upgrade->name[strlen(str->valuestring)] = 0;
		ROS_DEBUG("upgrade module name:%s",str->valuestring);
	}
	else
	{
	    return -1;
	}

	//read web item
	str = cJSON_GetObjectItem(json_data, "web");
    if(NULL == str)
    {
        //get object named "hello" faild
        return -1;
    }
	if(strlen(str->valuestring) < WEB_LEN)
	{	    
        memcpy(upgrade->web,str->valuestring,strlen(str->valuestring));
		upgrade->web[strlen(str->valuestring)] = 0;
		ROS_DEBUG("upgrade module web:%s",str->valuestring);
	}
	else
	{
	    return -1;
	}

	//read md5 item
	str = cJSON_GetObjectItem(json_data, "md5");
    if(NULL == str)
    {
        //get object named "hello" faild
        return -1;
    }
	if(strlen(str->valuestring) <= MD5_STRING_LEN)
	{ 
        memcpy(upgrade->md5,str->valuestring,strlen(str->valuestring));
		upgrade->md5[strlen(str->valuestring)] = 0;
		ROS_DEBUG("upgrade module md5:%s",str->valuestring);
	}
	else
	{
	    return -1;
	}

	//read force_update item
	str = cJSON_GetObjectItem(json_data, "force_update");
    if(NULL == str)
    {
        //get object named "hello" faild
        return -1;
    }
	if(strlen(str->valuestring) < FORCE_UPDATE_LEN)
	{   
        memcpy(upgrade->force_update,str->valuestring,strlen(str->valuestring));
		upgrade->force_update[strlen(str->valuestring)] = 0;
		ROS_DEBUG("upgrade module force_update:%s",str->valuestring);
	}
	else
	{
	    return -1;
	}

	//read version item
	str = cJSON_GetObjectItem(json_data, "version");
    if(NULL == str)
    {
        //get object named "hello" faild
        return -1;
    }
	if(strlen(str->valuestring) <= VERSION_LEN)
	{
        memcpy(upgrade->version,str->valuestring,strlen(str->valuestring));
		upgrade->version[strlen(str->valuestring)] = 0;
		ROS_DEBUG("upgrade module version:%s",str->valuestring);
	}
	else
	{
	    return -1;
	}
	
    return 0;
}

int get_navigation_product_id(char *str,int *idata)
{
    char tmp[BUF_LEN]={0,};
    char *p1 = NULL;
	
    if((NULL == str) || (NULL == idata))
	{
	    return -1;
	}

    //"2.1.0.0.1"
	p1 = strchr(str,'.');

	if((NULL == p1) || (p1-str-1 < 0))
	{
	    return -1;
	}
	memcpy(tmp,str,p1-str);
	tmp[p1-str] = 0;
	*idata = atoi(tmp);

    return 0;
}

int get_starline_product_id(char *str,int *idata)
{
    char tmp[BUF_LEN]={0,};
    char *p1 = NULL;
	
    if((NULL == str) || (NULL == idata))
	{
	    return -1;
	}

    //"1.3.1.1.5"
	p1 = strchr(str,'.');

	if((NULL == p1) || (p1-str-1 < 0))
	{
	    return -1;
	}
	memcpy(tmp,str,p1-str);
	tmp[p1-str] = 0;
	*idata = atoi(tmp);

    return 0;
}


int get_base_product_id(char *str,int *idata)
{
    char tmp[BUF_LEN]={0,};
    char *p1 = NULL;
	char *p2 = NULL;

	if((NULL == str) || (NULL == idata))
	{
	    return -1;
	}
	
    p1 = strchr(str,'M');
	p2 = strchr(str,'C');

	if((NULL == p1) || (NULL == p2) || (p1-p2-1 <= 0))
	{
	    return -1;
	}
	memcpy(tmp,str+1,p1-p2-1);
	tmp[p1-p2-1] = 0;
	*idata = atoi(tmp);
	
	return 0;
}

int get_sensor_product_id(char *str,int *idata)
{
    char tmp[BUF_LEN]={0,};
    char *p1 = NULL;
	char *p2 = NULL;

	if((NULL == str) || (NULL == idata))
	{
	    return -1;
	}
	
    p1 = strchr(str,'M');
	p2 = strchr(str,'C');
	if((NULL == p1) || (NULL == p2) || (p1-p2-1<= 0))
	{
	    return -1;
	}
	memcpy(tmp,str+1,p1-p2-1);
	tmp[p1-p2-1] = 0;
	*idata = atoi(tmp);
	
	return 0;
}

int get_led_power_product_id(char *str,int *idata)
{
    char tmp[BUF_LEN]={0,};
    char *p1 = NULL;
	char *p2 = NULL;

	if((NULL == str) || (NULL == idata))
	{
	    return -1;
	}
	
    p1 = strchr(str,'M');
	p2 = strchr(str,'C');
	if((NULL == p1) || (NULL == p2) || (p1-p2-1 <= 0))
	{
	    return -1;
	}
	memcpy(tmp,str+1,p1-p2-1);
	tmp[p1-p2-1] = 0;
	*idata = atoi(tmp);
	
	return 0;
}

int cmp_base_sub_version(char *p1,char *p2,int *m1,int *m2)
{
    char *p3 = NULL;
	char tmp1[128] = {0,};
	char tmp2[128] = {0,};
	int i = 0;
	int j = 0;

	if((NULL == p1) || (NULL == p2))
	{
	    return -1;
	}
	
    p3 = strpbrk(p1,"0123456789");
	if(NULL == p3)
	{
	    return -1;
	}
	i = p3-p1;
	if(i <= 0)
	{
	    return -1;
	}
	memcpy(tmp1,p1,i);

	p3 = strpbrk(p2,"0123456789");
	if(NULL == p3)
	{
	    return -1;
	}
	j = p3-p2;
	if(j <= 0)
	{
	    return -1;
	}
	memcpy(tmp2,p2,j);

    if(i != j)
	{
	    return -1;
	}
	for(i=0;i<j;i++)
	{
	    if(tmp1[i] != tmp2[i])
		{
		    return -1;
		}
	}

	//cmp version num
	p3 = strpbrk(p1,"0123456789");
	memcpy(tmp1,p3,strlen(p3));
	p3 = strpbrk(p2,"0123456789");
	memcpy(tmp2,p3,strlen(p3));

	*m1 = atoi(tmp1);
	*m2 = atoi(tmp2);
	if((0 == *m1) || (0 == *m2))
	{
	    return -1;
	}
    return 0;
}

int get_navigation_system_type_id(char *str,int *idata)
{
    char tmp[BUF_LEN]={0,};
    char *p1 = NULL;
	char *p2 = NULL;

	if((NULL == str) || (NULL == idata))
	{
	    return -1;
	}

    //"2.1.0.0.1"
	p1 = strchr(str,'.');
	p2 = strchr(p1+1,'.');
	if((NULL == p1) || (NULL == p2) || (p2-p1-1 <= 0))
	{
	    return -1;
	}
	memcpy(tmp,p1+1,p2-p1-1);
	tmp[p2-p1-1] = 0;
	*idata = atoi(tmp);
	
    return 0;
}

int get_navigation_version_code(char *str,int *idata)
{
    char tmp[BUF_LEN]={0,};
    char *p1 = NULL;
	char *p2 = NULL;
	int high_i = 0;
	int mid_i = 0;
	int low_i = 0;

	if((NULL == str) || (NULL == idata))
	{
	    return -1;
	}

	//"1.3.1.1.5"
	p1 = strchr(str,'.');
	p1 = strchr(p1+1,'.');
	p2 = strchr(p1+1,'.');
	if((NULL == p1) || (NULL == p2) || (p2-p1-1 <= 0))
	{
	    return -1;
	}
	memcpy(tmp,p1+1,p2-p1-1);
	tmp[p2-p1-1] = 0;
	high_i = atoi(tmp);

	p1 = p2;
	p2 = strchr(p1+1,'.');
	if((NULL == p1) || (NULL == p2) || (p2-p1-1 <= 0))
	{
	    return -1;
	}
	memcpy(tmp,p1+1,p2-p1-1);
	tmp[p2-p1-1] = 0;
	mid_i = atoi(tmp);
    if((p2-str+4) < (int)strlen(str))
	{
	    return -1;
	}
	low_i = atoi(p2+1);

    if((high_i > 255) || (mid_i > 255) || (low_i > 65535))
	{
	    return -1;
	}
	
	*idata = (high_i<<24)|(mid_i<<16)|(low_i);
    return 0;
}

int get_starline_system_type_id(char *str,int *idata)
{
    char tmp[BUF_LEN]={0,};
    char *p1 = NULL;
	char *p2 = NULL;

	if((NULL == str) || (NULL == idata))
	{
	    return -1;
	}

    //"1.3.1.1.5"
	p1 = strchr(str,'.');
	p2 = strchr(p1+1,'.');
	if((NULL == p1) || (NULL == p2) || (p2-p1-1 <= 0))
	{
	    return -1;
	}
	memcpy(tmp,p1+1,p2-p1-1);
	tmp[p2-p1-1] = 0;
	*idata = atoi(tmp);
	
    return 0;
}

int get_starline_version_code(char *str,int *idata)
{
    char tmp[BUF_LEN]={0,};
    char *p1 = NULL;
	char *p2 = NULL;
	int high_i = 0;
	int mid_i = 0;
	int low_i = 0;

	if((NULL == str) || (NULL == idata))
	{
	    return -1;
	}

	//"1.3.1.1.5"
	p1 = strchr(str,'.');
	p1 = strchr(p1+1,'.');
	p2 = strchr(p1+1,'.');
	if((NULL == p1) || (NULL == p2) || (p2-p1-1 <= 0))
	{
	    return -1;
	}
	memcpy(tmp,p1+1,p2-p1-1);
	tmp[p2-p1-1] = 0;
	high_i = atoi(tmp);

	p1 = p2;
	p2 = strchr(p1+1,'.');
	if((NULL == p1) || (NULL == p2) || (p2-p1-1 <= 0))
	{
	    return -1;
	}
	memcpy(tmp,p1+1,p2-p1-1);
	tmp[p2-p1-1] = 0;
	mid_i = atoi(tmp);
    if((p2-str+4) < (int)strlen(str))
	{
	    return -1;
	}
	low_i = atoi(p2+1);

    if((high_i > 255) || (mid_i > 255) || (low_i > 65535))
	{
	    return -1;
	}
	
	*idata = (high_i<<24)|(mid_i<<16)|(low_i);
    return 0;
}

int check_navigation_version(system_t *sys,module_upgrade_t *upgrade)
{
    int i = 0;
	int j = 0;
	int k = 0;

    if((NULL == sys) || (NULL == upgrade))
	{
	    return -1;
	}
	i = get_navigation_product_id((char *)upgrade->version,&j);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade product type is wrong");
	    return -2;
	}
	i = get_navigation_product_id((char *)sys->system_version,&k);
	if(0 != i)
	{
	    ROS_DEBUG("system product type is wrong");
	    return -2;
	}
	if(k != j)
	{
	    ROS_DEBUG("upgrade product type does not match!");
	    return -3;
	}

	i = get_navigation_system_type_id((char *)upgrade->version,&j);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade system_type is wrong!");
	    return -4;
	}
	i = get_navigation_system_type_id((char *)sys->system_version,&k);
	if(0 != i)
	{
	    ROS_DEBUG("system system_type is wrong!");
	    return -4;
	}
    if(SYSTEM_TYPE_ID != j)
   	{
        ROS_DEBUG("upgrade system type does not match!");
	    return -5;
    }
	
    i = get_navigation_version_code((char *)upgrade->version,&j);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade version code could not get!");
		return -6;
	}
    i = get_navigation_version_code((char *)sys->system_version,&k);
	if(0 != i)
	{
	    ROS_DEBUG("system version code could not get!");
		return -6;
	}
	if(j <= k)
	{
	    return -7;
	}
    return 0;
}

int check_starline_version(system_t *sys,module_upgrade_t *upgrade)
{
    int i = 0;
	int j = 0;
	
    if((NULL == sys) || (NULL == upgrade))
	{
	    return -1;
	}

	i = get_starline_product_id((char *)upgrade->version,&j);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade product type is wrong");
	    return -2;
	}

	if(PRODUCT_ID != j)
	{
	    ROS_DEBUG("upgrade product type does not match!");
	    return -3;
	}

	i = get_starline_system_type_id((char *)upgrade->version,&j);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade system type is wrong!");
	    return -4;
	}
    if(SYSTEM_TYPE_ID != j)
   	{
        ROS_DEBUG("upgrade system type does not match!");
	    return -5;
    }
	
    i = get_starline_version_code((char *)upgrade->version,&j);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade version code could not get!");
		return -6;
	}

	if(j <= OFFICIAL_VERSION_CODE)
	{
	    return -7;
	}
    return 0;
}


int check_base_version(system_t *sys,module_upgrade_t *upgrade)
{
	char *p1 = NULL;
	char *p2 = NULL;
	int i = 0;
	int j = 0;
	int k = 0;

	if((NULL == sys) || (NULL == upgrade))
	{
	    return -1;
	}

	if(2 != sys->base.work_normal)
	{
	    return -2;
	}

	i = get_base_product_id((char *)sys->base.version,&j);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade cannot get base version");
	    return -3;
	}
	i = get_base_product_id((char *)upgrade->version,&k);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade cannot get base version");
	    return -4;
	}
	if(j != k)
	{
	    ROS_DEBUG("upgrade product type does not match!");
	    return -5;
	}

	p1 = strstr((char *)sys->base.version,"M06");
	p2 = strstr((char *)upgrade->version,"M06");
	if((NULL == p1) || (NULL == p2))
	{
	    ROS_DEBUG("upgrade module id is wrong!");
	    return -6;
	}

	p1 = p1 + strlen("M06");
	p2 = p2 + strlen("M06");
	k = cmp_base_sub_version(p1,p2,&i,&j);
	
	if(0 != k)
	{
	    return -7;
	}

	if(j <= i)
	{
	    return -8;
	}
    return 0;
}

int check_sensor_version(system_t *sys,module_upgrade_t *upgrade)
{
	char *p1 = NULL;
	char *p2 = NULL;
	int i = 0;
	int j = 0;
	int k = 0;

	if((NULL == sys) || (NULL == upgrade))
	{
	    return -1;
	}

	if(2 != sys->sensor.work_normal)
	{
	    return -2;
	}

	i = get_sensor_product_id((char *)sys->sensor.version,&j);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade cannot get sensor version");
	    return -3;
	}
	i = get_sensor_product_id((char *)upgrade->version,&k);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade cannot get sensor version");
	    return -4;
	}
	if(j != k)
	{
	    ROS_DEBUG("upgrade product type does not match!");
	    return -5;
	}

	p1 = strstr((char *)sys->sensor.version,"M04");
	p2 = strstr((char *)upgrade->version,"M04");
	if((NULL == p1) || (NULL == p2))
	{
	    ROS_DEBUG("upgrade module id is wrong!");
	    return -6;
	}

	p1 = p1 + strlen("M04");
	p2 = p2 + strlen("M04");
	k = cmp_base_sub_version(p1,p2,&i,&j);
	
	if(0 != k)
	{
	    return -7;
	}

	if(j <= i)
	{
	    return -8;
	}
    return 0;
}

int check_led_power_version(system_t *sys,module_upgrade_t *upgrade)
{
	char *p1 = NULL;
	char *p2 = NULL;
	int i = 0;
	int j = 0;
	int k = 0;

	if((NULL == sys) || (NULL == upgrade))
	{
	    return -1;
	}

	if(2 != sys->led_power.work_normal)
	{
	    return -2;
	}

	i = get_led_power_product_id((char *)sys->led_power.version,&j);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade cannot get led power version");
	    return -3;
	}
	i = get_led_power_product_id((char *)upgrade->version,&k);
	if(0 != i)
	{
	    ROS_DEBUG("upgrade cannot get led power version");
	    return -4;
	}
	if(j != k)
	{
	    ROS_DEBUG("upgrade product type does not match!");
	    return -5;
	}

	p1 = strstr((char *)sys->led_power.version,"M07");
	p2 = strstr((char *)upgrade->version,"M07");
	if((NULL == p1) || (NULL == p2))
	{
	    ROS_DEBUG("upgrade module id is wrong!");
	    return -6;
	}

	p1 = p1 + strlen("M07");
	p2 = p2 + strlen("M07");
	k = cmp_base_sub_version(p1,p2,&i,&j);
	
	if(0 != k)
	{
	    return -7;
	}

	if(j <= i)
	{
	    return -8;
	}
    return 0;
}


int check_force_upgrade(unsigned char *str)
{
    int i = 0;
    if((0 == strcmp((char *)str,"Y")) || (0 == strcmp((char *)str,"y")))
	{
	    i = 1;
	}
	else
	{
	    i = 0;
	}
	return i;
}
//for new download program
int check_upgrade_system(system_t *sys,env_t *env,unsigned char *data,int *force)
{
    int i = 0;
	ans_status_t ans;
    module_upgrade_t upgrade;
	unsigned char md5_upper_value[MD5_SIZE] = {0,};
	
    //read upgrade json context
    i = read_upgrade_json_info(&upgrade,data);
    ROS_DEBUG("check_upgrade_system read:%d",i);
	if(0 != i)
	{
	    ans.level = LEVEL_INFO;
		ans.module = MODULE_NAV;
		ans.function = 27;
		ans.len = 60;
		set_int_buf(ans.data,7);
		set_int_buf(ans.data+4,MODULE_NAV);
		set_int_buf(ans.data+8,-1);
		set_event_buffer(&ans);
	    return -1;
	}
	//check force update or now
    *force = check_force_upgrade(upgrade.force_update);

	i = md5_string_to_hex(upgrade.md5,md5_upper_value);
    ROS_DEBUG("check_upgrade_system md5:%d",i);
	if(0 != i)
	{
	    ROS_DEBUG("md5 string to hex error");
		ans.level = LEVEL_INFO;
		ans.module = MODULE_NAV;
		ans.function = 27;
		ans.len = 60;
		set_int_buf(ans.data,7);
		set_int_buf(ans.data+4,MODULE_NAV);
		set_int_buf(ans.data+8,-2);
		set_event_buffer(&ans);
	    return -1;
	}
	
	//decide what to do,based on module
	if(0 == strcmp((char *)upgrade.name,"navigation"))
	{
	    //check upgrade or not
	    i = check_navigation_version(sys,&upgrade);
        ROS_DEBUG("check_upgrade_system version navigation:%d",i);
		if(0 != i)
		{
		    //report could not upgrade event,then return;
		    ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_NAV);
		    set_int_buf(ans.data+8,i-2);
		    set_event_buffer(&ans);
		    return -1;
		}

		//set to download upgrade file
		i = set_avalible_cloud_buf(&upgrade);
		if(i < 0)
		{
            ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_NAV);
		    set_int_buf(ans.data+8,-10);
		    set_event_buffer(&ans);
			return -1;
		}

		sys->navigation_download_index = i;
		sys->navigation_upgrade_status = 1;//wait for downloading finish
		memcpy(sys->navigation_md5,md5_upper_value,MD5_SIZE);
	}
	else if(0 == strcmp((char *)upgrade.name,"starline"))
	{
	    //check upgrade or not
	    i = check_starline_version(sys,&upgrade);
        ROS_DEBUG("check_upgrade_system version starline:%d",i);
		if(0 != i)
		{
		    //report could not upgrade event,then return;
		    ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_NAV);
		    set_int_buf(ans.data+8,i-2);
		    set_event_buffer(&ans);
		    return -1;
		}

		//set to download upgrade file
		i = set_avalible_cloud_buf(&upgrade);
		if(i < 0)
		{
            ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_NAV);
		    set_int_buf(ans.data+8,-10);
		    set_event_buffer(&ans);
			return -1;
		}

		sys->starline_download_index = i;
		sys->starline_upgrade_status = 1;//wait for downloading finish
		memcpy(sys->starline_md5,md5_upper_value,MD5_SIZE);
	}
	else if(0 == strcmp((char *)upgrade.name,"base"))
	{
	    //check could upgrade or not,compare version info
	    i = check_base_version(sys,&upgrade);
        ROS_DEBUG("check_upgrade_system version base:%d",i);
		if(0 != i)
		{
		    //report could not upgrade event,then return;
		    ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_BASE);
		    set_int_buf(ans.data+8,i);
		    set_event_buffer(&ans);
		    return -1;
		}

		//set to download upgrade file
		i = set_avalible_cloud_buf(&upgrade);
		if(i < 0)
		{
            ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_BASE);
		    set_int_buf(ans.data+8,-9);
		    set_event_buffer(&ans);
			return -1;
		}

		//set movebase comunication module to wait upgrade
		sys->base.download_index = i;
		sys->base.upgrade_status = 1;
		memcpy(sys->base.upgrade_md5,md5_upper_value,MD5_SIZE);
	}
	else if(0 == strcmp((char *)upgrade.name,"sensor"))
	{
	    //check could upgrade or not,compare version info
	    i = check_sensor_version(sys,&upgrade);
        ROS_DEBUG("check_upgrade_system version sensor:%d",i);
		if(0 != i)
		{
		    //report could not upgrade event,then return;
		    ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_SENSOR);
		    set_int_buf(ans.data+8,i);
		    set_event_buffer(&ans);
		    return -1;
		}

		//set to download upgrade file
		i = set_avalible_cloud_buf(&upgrade);
		if(i < 0)
		{
            ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_SENSOR);
		    set_int_buf(ans.data+8,-9);
		    set_event_buffer(&ans);
			return -1;
		}

		//set sensor comunication module to wait upgrade
		sys->sensor.download_index = i;
		sys->sensor.upgrade_status = 1;
		memcpy(sys->sensor.upgrade_md5,md5_upper_value,MD5_SIZE);
	}
	else if(0 == strcmp((char *)upgrade.name,"power"))
	{
	    //check could upgrade or not,compare version info
	    i = check_led_power_version(sys,&upgrade);
        ROS_DEBUG("check_upgrade_system version power:%d",i);
		if(0 != i)
		{
		    //report could not upgrade event,then return;
		    ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_POWER);
		    set_int_buf(ans.data+8,i);
		    set_event_buffer(&ans);
		    return -1;
		}

		//set to download upgrade file
		i = set_avalible_cloud_buf(&upgrade);
		if(i < 0)
		{
            ans.level = LEVEL_INFO;
		    ans.module = MODULE_NAV;
		    ans.function = 27;
		    ans.len = 60;
		    set_int_buf(ans.data,7);
		    set_int_buf(ans.data+4,MODULE_POWER);
		    set_int_buf(ans.data+8,-9);
		    set_event_buffer(&ans);
			return -1;
		}

		//set power comunication module to wait upgrade
		sys->led_power.download_index = i;
		sys->led_power.upgrade_status = 1;
		memcpy(sys->led_power.upgrade_md5,md5_upper_value,MD5_SIZE);
	}
    return 0;
}

