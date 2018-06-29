#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <stdio.h>
#include <curl/curl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "../include/starline/config.h"
#include "../include/starline/system.h"
#include "../include/starline/report.h"
#include "../include/starline/handle_command.h"
#include "../include/starline/md5.h"

#include "../include/starline/cloud.h"

static cloud_t gcloud;

void reset_cloud_params(void)
{
    int i = 0;
	
    memset(&gcloud,0,sizeof(cloud_t));
	for(i=0;i<DOWNLOAD_BUF_SIZE;i++)
	{
	    gcloud.download_buf[i].download_status = FINISHED;
	}
	return;
}
int get_cloud_flag(void)
{
    return gcloud.cloud_flag;
}

download_t *get_download_buf(int i)
{
    download_t *p = NULL;
    if((i < 0) || (i >= DOWNLOAD_BUF_SIZE))
	{
	    return p;
	}

	p = &(gcloud.download_buf[i]);
	return p;
}
int set_avalible_cloud_buf(module_upgrade_t *upgrade)
{
	int i = 0;

	for(i=0;i<DOWNLOAD_BUF_SIZE;i++)
	{
		if((FINISHED == gcloud.download_buf[i].download_status)
			&& (0 == gcloud.download_buf[i].result_flag))
		{
			gcloud.download_buf[i].download_status = WAIT_DOWNLOAD;
			memcpy(&(gcloud.download_buf[i].module),upgrade,sizeof(module_upgrade_t));
			return i;
		}
	}
	
	return -1;
}

static int check_download_files(cloud_t *sys)
{
    int i = 0;
	
    if(NULL != sys)
	{
	    for(i=0;i<DOWNLOAD_BUF_SIZE;i++)
		{
		    if((WAIT_DOWNLOAD == sys->download_buf[i].download_status) ||
			(DOWNLOADING == sys->download_buf[i].download_status))
			{
			    return 1;
			}
		}
	}
	return 0;
}
static int write_data(char *buffer,size_t size, size_t nitems,void *outstream)
{
    int written = fwrite(buffer, size, nitems, (FILE*)outstream);
    return written;
}

static int cal_file_md5(download_t *download)
{
    FILE* fd = NULL;
	char path[128]="/home/robot/catkin_ws/download/";
	unsigned char md5_cal_value[MD5_SIZE] = {0,};
	unsigned char md5_upper_value[MD5_SIZE] = {0,};
	int i = 0;
	int file_size = 0;
	unsigned char *tmp = NULL;

	strcat(path,(char *)download->module.name);
    fd = fopen(path, "rb" );
	if(NULL == fd)
	{
	    ROS_ERROR("open download file failed!");
	    return -1;
	}
	fseek(fd,0,SEEK_END);
	file_size = ftell(fd);
	fseek(fd,0,SEEK_SET);
	tmp = (unsigned char *)malloc(file_size*sizeof(char));
	i = fread(tmp,sizeof(char),file_size,fd);
    fclose(fd);
    if(0 > i)
	{
	    ROS_ERROR("fread file_size failed!");
		return -1;
	}
	i = compute_md5(tmp,file_size,md5_cal_value);
	if(0 != i)
	{
	    ROS_ERROR("compute file md5 failed!");
		return -1;
	}

	i = md5_string_to_hex(download->module.md5,md5_upper_value);
	if(0 != i)
	{
	    ROS_ERROR("md5 string to hex error");
	    return -1;
	}

	for(i = 0;i<MD5_SIZE;i++)
	{
	    if(md5_cal_value[i] != md5_upper_value[i])
		{
		    return -1;
		}
	}

    return 0;
}
static int download_file(download_t *download)
{
    int i = 0;
	CURLcode r = CURLE_GOT_NOTHING;
	CURL* curl = NULL;
	FILE* fd = NULL;
	char path[128]="/home/robot/catkin_ws/download/";
	
	if(NULL == download)
	{
	    return -1;
	}

    //check download folder and create it,if does not exist.
    i = access(path,F_OK);
	if(0 != i)
	{
	    i = mkdir(path,S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH);
		if(0 != i)
		{
		    ROS_ERROR("download folder can not create");
		    return -1;
		}
	}

	//download file
    curl = curl_easy_init();
	strcat(path,(char *)download->module.name);
    fd = fopen(path, "wb" );
	if(NULL == fd)
	{
	    curl_easy_cleanup(curl);
		ROS_ERROR("open upgrade file:%s failed!",path);
		return -1;
	}

    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)fd );
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 1);  // 
    //curl_easy_setopt(curl, CURLOPT_URL, "www.google.com.hk");
    //curl_easy_setopt(curl, CURLOPT_URL, "http://www.myee.online/push/100/driver/v0.0.8/starline");
    curl_easy_setopt(curl, CURLOPT_URL, download->module.web);
    r = curl_easy_perform(curl);
    fclose(fd);
    curl_easy_cleanup(curl);
    if(CURLE_OK != r)
    {
        ROS_ERROR("curl error:%s",curl_easy_strerror(r));
		return -1;
    }
	if(0 == strcmp((char *)download->module.name,"starline"))
	{
	    i = chmod(path,S_IRWXU|S_IRWXG|S_IROTH|S_IWOTH|S_IXOTH);
		if(0 != i)
		{
		    ROS_ERROR("chmod starline failed");
		    return -1;
		}
	}
	return 0;
}

static void handle_download(cloud_t *sys)
{
    int i = 0;
	int flag = 0;

	for(i=0;i<DOWNLOAD_BUF_SIZE;i++)
	{
	    if((WAIT_DOWNLOAD == sys->download_buf[i].download_status) ||
			(DOWNLOADING == sys->download_buf[i].download_status))
		{
		    flag = download_file(&(sys->download_buf[i]));
			if(0 == flag)
			{
			    //cal file md5
			    flag = cal_file_md5(&(sys->download_buf[i]));
				if(0 != flag)
				{
				    //report download module file failed!
				    sys->download_buf[i].result_flag = 1;
				}
				else
				{
				    //report download module file success!
				    sys->download_buf[i].result_flag = 0;
				}
			}
			else
			{
			    //report download module file failed!
			    sys->download_buf[i].result_flag = 1;
			}
			
			sys->download_buf[i].download_status = FINISHED;
		}
	}
    return;
}

void *cloud_com_thread_start(void *)
{
    int tmp = 0;
    ros::Rate loop_rate(1.0);
    while(ros::ok())
	{
	    gcloud.cloud_flag = 1;

		//check need download upgrade files
		tmp = check_download_files(&gcloud);
        if(1 == tmp)
		{
		    handle_download(&gcloud);
		}
	    ros::spinOnce();
        loop_rate.sleep();
	}
	gcloud.cloud_flag = 0;
    return 0;
}


