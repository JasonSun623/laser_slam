#include "ros/ros.h"
#include <stdio.h>      
#include <stdlib.h>     
#include <unistd.h>     
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      
#include <termios.h>   
#include <errno.h>     
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

#include "../include/starline/config.h"
#include "../include/starline/sensor.h"

#include "../include/starline/json.hpp"
#include "std_msgs/String.h"
static std::string laser_frames[LASER_NUM - 3] = {"laser_frame_0","laser_frame_1","laser_frame_2","laser_frame_3","laser_frame_4","laser_frame_5",
										      "laser_frame_6","laser_frame_7","laser_frame_8","laser_frame_9"};


using json = nlohmann::json;
static sensor_sys_t sensor_sys;
static sensor_info_t sensor_info;
static int sensor_over_time_flag = 0;
static int last_unread_bytes = 0;
static unsigned char recv_buf_last[BUF_LEN] = {0};
ros::Publisher hall_pub;
//laser data:mm to m
static inline double restore_laser_len(unsigned char *buf)
{
    int i = 0;
    double len = 0.4;
  
    i = (((int)buf[1])<<8)|((int)buf[0]);
    len = ((double)i)*LEN_MM_TO_M;
    return len;
}

//sonar data:cm to m
static inline double restore_sonar_len(unsigned char *buf)
{
    int i = 0;
    double len = 1.0;
  
    i = (int)buf[0];
    len = ((double)i)*LEN_CM_TO_M;
    return len;
}

//sensor data:cm to m
static inline double restore_sensor_data(unsigned char *buf)
{
    int i = 0;
    double data = 2.0;
  
    i = (int)buf[0];
    data = ((double)i)*LEN_CM_TO_M;
    return data;
}

static void pub_laser_data(sensor_sys_t *sys)
{
	sys->laser_data.header.stamp = ros::Time::now();
	sys->laser_data.header.frame_id = "laser";
	sys->laser_data.radiation_type = INFRARED;
	sys->laser_data.field_of_view = 0.1;
	sys->laser_data.min_range =  0.0;
	sys->laser_data.max_range = 0.4;


sensor_msgs::PointCloud2 cloud_out;
	cloud_out.header.stamp = ros::Time::now();
	cloud_out.height = 1;
	cloud_out.width  = 1;
	cloud_out.fields.resize (4);
	cloud_out.fields[0].name = "x";
	cloud_out.fields[0].offset = 0;
	cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	cloud_out.fields[0].count = cloud_out.width;
	cloud_out.fields[1].name = "y";
	cloud_out.fields[1].offset = 4;
	cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	cloud_out.fields[1].count = cloud_out.width;
	cloud_out.fields[2].name = "z";
	cloud_out.fields[2].offset = 8;
	cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	cloud_out.fields[2].count = cloud_out.width;
	cloud_out.fields[3].name = "intensity";
	cloud_out.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
	cloud_out.fields[3].offset = 12;
	cloud_out.fields[3].count = cloud_out.width;
    cloud_out.point_step = 16;
	cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
	cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
	cloud_out.is_dense = false;



	bool block_back =false;
	ros::param::get("block_back",block_back);
	for(int i=0;i<LASER_NUM;i++)
	{

		if(i < LASER_NUM - 3)//bu yao dong
		{

			if(block_back)
			{
				if(i != 6 && i != 9)
				{
					cloud_out.header.frame_id = laser_frames[i];
					float *pstep = (float*)&cloud_out.data[0];
					pstep[0] = (float)sys->laser_len[i];
					pstep[1] = 0.0;
					sys->lasercloud_pub.publish(cloud_out);
				}
			}
			else
			{
				cloud_out.header.frame_id = laser_frames[i];
				float *pstep = (float*)&cloud_out.data[0];
				pstep[0] = (float)sys->laser_len[i];
				pstep[1] = 0.0;
				sys->lasercloud_pub.publish(cloud_out);
			}

		}	    
				
		sys->laser_data.range.push_back(sys->laser_len[i]);
	}
	sys->sensor_pub.publish(sys->laser_data);
	sys->laser_data.range.clear();

}

static void pub_sonar_data(sensor_sys_t *sys)
{
	sys->sonar_data.header.stamp = ros::Time::now();
	sys->sonar_data.header.frame_id = "sonar";
	sys->sonar_data.radiation_type = ULTRASOUND;
	sys->sonar_data.field_of_view = 0.1;
	sys->sonar_data.min_range = 0.4;
	sys->sonar_data.max_range = 1.9;

	for(int i=0;i<SONAR_NUM;i++)
		sys->sonar_data.range.push_back(sys->sonar_len[i]);

	sys->sensor_pub.publish(sys->sonar_data);

	sys->sonar_data.range.clear();
}

void pub_hall_msg(const nlohmann::json j_msg)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    pub_json_msg.data = ss.str();
    hall_pub.publish(pub_json_msg);
}
static void pub_hall_data(sensor_sys_t *sys)
{
	static bool state[2] = {false, false};
	json j;
	//static uint8_t flag = 0;
	//if(flag == 0)
	{
		//state[0] = false;
		//state[1] = false;
		//flag = 1;
	}
	
	if((state[0] != sys->hall_state[0]) || (state[1] != sys->hall_state[1]))
	{
		state[0] = (bool)sys->hall_state[0];
		state[1] = (bool)sys->hall_state[1];

		j.clear();
		j = 
		{
			{"sensor_name","hall_sensor"},
				{"data",{

					{"hall_1", (bool)sys->hall_state[0]},
					{"hall_2", (bool)sys->hall_state[1]},
				},
			},
		};
		pub_hall_msg(j);
	}
	
    
}

static void handle_rev_frame(sensor_sys_t *sys,unsigned char * frame_buf)
{
    int frame_len = 0;
	int i = 0;
	int j = 0;
	unsigned char check_data = 0;
	frame_len = frame_buf[1];

    for(i=0;i<frame_len-2;i++)
    {
        check_data += frame_buf[i];
    }
    
	if(check_data != frame_buf[frame_len-2] || 0xA5 != frame_buf[frame_len -1])
	{
        ROS_INFO("sensor receive frame check error");
		return;
	}
    //for(i =0;i<frame_len;i++)
    //{
    //    ROS_INFO("sensor receive:%d",frame_buf[i]);
    //}
	if(2 != sys->work_normal)
    {
         sys->work_normal = 2;
    }
	if(0 == sys->handle_data_flag)
    {
         sys->handle_data_flag = 1;
		 sensor_info.recv_type = frame_buf[2];

		 switch (sensor_info.recv_type)
		 {
             case 0x01:
                sensor_info.set_safe_distance_ack = frame_buf[3]; 
                break;
                                 
			 case 0x02:
				for(j=0; j<10; j++)
				{
                     sensor_info.safe_distance_set[j] = frame_buf[3+j]; 
				}
				sensor_sys.estop_fb_limit = sensor_info.safe_distance_set[0]*LEN_CM_TO_M;
                break;
								 
			 case 0x03:
				sys->infrared_flag = frame_buf[3];

                ROS_INFO("sensor 0x03!!!!!");
				for(j = 0;j < LASER_NUM;j++)
                {
                    sys->laser_len[j] = restore_sensor_data(&frame_buf[3+j]);
                    //ROS_INFO("sensor receive:%d",sys->laser_len[j]);
                }
				pub_laser_data(sys);
				for(j = 0;j < SONAR_NUM;j++)
				{
					sys->sonar_len[j] = restore_sensor_data(&frame_buf[3+LASER_NUM+j]);
				}
				pub_sonar_data(sys);
                for(j = 0; j < HALL_NUM; j++)
                {
                    //if((frame_buf[3+SONAR_NUM+LASER_NUM+j] == 1)&& (frame_buf[3+SONAR_NUM+LASER_NUM+j] == 0))
                    {
                        sys->hall_state[j] = frame_buf[3+SONAR_NUM+LASER_NUM+j];
                        ROS_INFO("hall %d state is %d",j, frame_buf[3+SONAR_NUM+LASER_NUM+j]);
                    }
                }
                pub_hall_data(sys);
                break;
								 
			 case 0x0D:
                sensor_info.function_cali_cmd_rlt = (int)frame_buf[3];
	            sensor_info.function_cali_param_rlt = (int)frame_buf[4];
				break;

			 case 0x0E:
                for(j=0; j<SENSOR_HARDWARE_VER_LEN; j++)
				{
                    sys->hardware_version[j] = frame_buf[3+j]; 
				}
				for(j=0; j<SENSOR_SOFTWARE_VER_LEN; j++)
				{
                    sys->software_version[j] = frame_buf[7+j]; 
				}
				break;

			 case 0x0F:
				sensor_info.upgrade_type = (int)frame_buf[3];
				switch(sensor_info.upgrade_type)
				{
                      case 0x00:
                         sensor_info.upgrade_ready_rlt = (int)frame_buf[4];
						 break;
										  
					  case 0x01:                    
						 sensor_info.upgrade_recv_rlt = (int)frame_buf[4];
						 break;

					  case 0x02:
						 sensor_info.upgrade_end_rlt = (int)frame_buf[4];
                         break;

					  default:
                          break;
				}
                break;
				
			 default:
                break;
		}
        sys->handle_data_flag = 0;
        sys->rec_num++;
    }
}

static int handle_receive_data(sensor_sys_t *sys)
{
    int nread = 0;
    int i = 0;
	int j = 0;
    int data_Len = 0;
    int frame_len = 0;
    unsigned char recv_buf[BUF_LEN] = {0};
	unsigned char recv_buf_complete[BUF_LEN] = {0};
    unsigned char recv_buf_temp[BUF_LEN] = {0};

	struct stat file_info;

    ROS_INFO("in FUNC   !!!!");
    
    if(NULL == sys)
    {
        ROS_INFO("sys NULL!");
        return -1;
    }
    if(COM_RUN_OK != sys->com_state && COM_CHECK_VERSION != sys->com_state)
    {
        ROS_INFO("sensor_handle_receive_data: com_state != COM_RUN_OK && COM_CHECK_VERSION");
        return -1;
    }
    if(0 != last_unread_bytes)
    {
        for(j=0;j<last_unread_bytes;j++)
        {
            recv_buf_complete [j] = recv_buf_last[j];
        }
    }
    if((nread = read(sys->com_device, recv_buf, BUF_LEN))>0)
    { 
        //ROS_INFO("sensor nread:%d",nread);
        //ROS_INFO("sensor last_unread_bytes:%d",last_unread_bytes);
        ROS_INFO("get %d!",nread);
        memcpy(recv_buf_complete+last_unread_bytes,recv_buf,nread);
        data_Len = last_unread_bytes + nread;
        last_unread_bytes = 0;

        while(i<data_Len)
        {
//            ROS_INFO("rcv_buf[%d]:%2x",i,recv_buf_complete[i]);
            if(0x5A == recv_buf_complete [i])
            {
                
                 frame_len = recv_buf_complete[i+1]; 
                 ROS_INFO("frame_len:%d",frame_len);
                 if(i+frame_len <= data_Len)
                 {
                      if(0xA5 == recv_buf_complete[i+frame_len-1])
                      {
                           for(j=0;j<frame_len;j++)
                           {
                               recv_buf_temp[j] = recv_buf_complete[i+j];
                               ROS_INFO("recv_buf_temp:%x",recv_buf_temp[j]);
                           }
                           handle_rev_frame(sys,recv_buf_temp);
                           i = i+ frame_len;
                      }
                      else
                      {
                           i++;
                      }
                 }
                 else
                 {
                      last_unread_bytes = data_Len - i;
                      for(j=0;j<last_unread_bytes;j++)
                      {
                           recv_buf_last[j] = recv_buf_complete[i+j];
                           ROS_INFO("out get!");
                      }
                      break;
                 }
            }
            else 
            {
                 i++;
            }
        }
    
    }
    else 
    {
        i = stat(sys->dev,&file_info);
        if(-1 == i)
        {
            sys->com_state = COM_CLOSING;
        }
    }
    return 0;
}

static int send_serial(unsigned char *send_buf,sensor_sys_t *sys)
{
    int len = 0;

	int send_buf_len = 0;

    if((NULL == sys) || (NULL == send_buf))
    {
        ROS_INFO("sys or send_buf NULL!");
        return -1;
    }
    
    if(COM_RUN_OK != sys->com_state && COM_CHECK_VERSION != sys->com_state)
    {
        ROS_INFO("sensor_send_serial: com_state != COM_RUN_OK && COM_CHECK_VERSION != sys->com_state");
        return 0;
    }

    send_buf_len = send_buf[1];
	if(send_buf_len <= 0)
	{
        ROS_INFO("sensor send_buf len: %d small 0!",send_buf_len);
        return -1;
	}
for(int i =0;i<send_buf_len;i++){
 //      ROS_INFO("sensor_send_buf :%02x",send_buf[i]);
}
    len = write(sys->com_device,send_buf,send_buf_len);
    if (len == send_buf_len)
    {
         //ROS_INFO("sensor send ok");
         return 0;
    }     
    else   
    {               
        tcflush(sys->com_device,TCOFLUSH);
        if(-1 == len)
        {
            sys->com_state = COM_CLOSING;
        }
        return -1;
    }
}

void get_sensor_version(void)
{
	unsigned char data[6]={0};
	
	data[0] = 0x5A;
	data[1] = 0x06;
	data[2] = 0x0E;
	data[3] = 0;
	data[4] = data[0]+data[1]+data[2]+data[3];
	data[5] = 0xA5;
	send_serial(data,&sensor_sys);
    usleep(SENSOR_SLEEP_TIME);
	handle_receive_data(&sensor_sys);
}

static bool check_version(void)
{
    int i = 0;
    for(i=0; i<SENSOR_HARDWARE_VER_LEN; i++)
	{
        if('\0' == sensor_sys.hardware_version[i])
        {
             return false;
        }            
	}
	for(i=0; i<SENSOR_SOFTWARE_VER_LEN; i++)
	{
         if('\0' == sensor_sys.software_version[i]) 
         {
			 return false;
         }
    } 
    return true;
}

static void sensor_init (void)
{
    char com_device_path[]="/dev/ros/sensor";
    sensor_sys.work_normal = 0;
    
    sensor_sys.com_rssi = 0;
    sensor_sys.handle_data_flag = 0;
	sensor_sys.estop_limit = 0.45;
    memcpy(sensor_sys.dev,com_device_path,sizeof(com_device_path));

    sensor_info.set_safe_distance_ack = -1;
	sensor_info.function_cali_cmd_rlt = -1;
	sensor_info.function_cali_param_rlt = -1;
	sensor_info.set_function_cali_ack = -1;
	sensor_info.upgrade_ready_rlt = -1;
	sensor_info.upgrade_recv_rlt = -1;
	sensor_info.upgrade_end_rlt = -1;
}

static void update_system_state(sensor_sys_t *sys)
{
    static int last_file_flag = 0;
    struct stat file_info;
    int i = 0;

    if(NULL == sys)
    {
        ROS_INFO("sys NULL!");
        return;
    }
    
    switch(sys->com_state)
    {
        case COM_OPENING:
            
            sys->work_normal = 0;
            sensor_init();
            i = stat(sys->dev,&file_info);
            if(-1 == i)
            {
                if(-1 != last_file_flag)
                {
                    ROS_INFO("sensors com device does not exist\n");
                }
                last_file_flag = i;
                return;
            }
            last_file_flag = i;
            sys->com_device = open_com_device(sys->dev);
            if(-1 != sys->com_device)
            {
                sys->com_state =  COM_RUN_OK;
                sys->com_rssi= 4;
				sys->work_normal = 1;
                ROS_INFO("open sensors com success!");
            }
            else
            {
                ROS_INFO("open sensors com device failed");
                return;
            }
            
            set_speed(sys->com_device,115200);
            set_parity(sys->com_device,8,1,'N');/* */ 
            break;
		
        case COM_CHECK_VERSION:
            get_sensor_version();
            if(check_version())
            {
                 sys->com_state = COM_RUN_OK;
            }
            break;
        	
        case COM_RUN_OK:
			//ROS_INFO("open com device:%d ok\n",sys->com_device);
            break;
            
        case COM_CLOSING:
            close(sys->com_device);
            //usleep(CLOSE_WAIT_TIME);
            sys->com_state = COM_OPENING;
            sys->com_rssi = 0;
            ROS_INFO("close com device:%d\n",sys->com_device);
            break;  
			
        default:
            break;
    }
    return;
}

static long get_upgrade_size(char* path)
{
	FILE *file;
	
	long len = 0;
	
	file = fopen(path,"rb");
	if(NULL == file)
	{
		ROS_INFO("get upgrade size failed");
        return 0;
	}
	
	fseek(file,0,SEEK_END);
	len = ftell(file);
	fclose(file);	
	return len;
}

static int send_ready_upgrade(char * path, char * md5char)
{
	unsigned char data[26] = {0};
    union {
        unsigned long lnum;
        unsigned char cnum[4];
    }filesize;
	int i = 0;
	
	data[0] = 0x5A;
	data[1] = 0x1A;
	data[2] = 0x0F;
	data[3] = 0x00;
											 
	for(i=0; i<16; i++)
	{
		data[i+4] = md5char[i];
	}
  
    //big end or small end convert socket frame
    filesize.lnum = htonl(get_upgrade_size(path));
	if(0 == filesize.lnum){
       ROS_INFO("sensor upgrade file is 0\n");   
	   return -1;
	}

	for(i=0; i<4; i++)
	{
        data[i+20] = filesize.cnum[i];
	}

	for(i=0; i<24; i++)
	{
		data[24] += data[i];
	}
	data[25] = 0xA5;  
	send_serial(data,&sensor_sys);
	usleep(SENSOR_READY_UPGRADE_SLEEP_TIME);
	handle_receive_data(&sensor_sys);
    ROS_INFO("send_ready_upgrade over");
	return 0;
}

static int send_upgrade_file_frame(char* buffer,int len)
{
     unsigned char data[SENSOR_UPGRADE_FILE_FRAME_LEN+6]={0};
	 int i = 0;
	 
	 data[0] = 0x5A; 
	 data[1] = len+6;
	 data[2] = 0x0F;
	 data[3] = 0x01;
	 for(i=0; i<len; i++)
	 {
        data[i+4] = buffer[i];
	 }
	 for(i=0; i<len+4; i++)
     {
	    data[len+4] += data[i];
	 }
     data[len+5] = 0xA5;     
	 send_serial(data,&sensor_sys); 
	 return 0;
}

static void send_file_over_time(int sig)
{    
    sensor_over_time_flag =1;
}

static int send_upgrade_file(char* path)
{	
	char buffer[SENSOR_UPGRADE_FILE_FRAME_LEN]={0};
	FILE *file;
    int len = 0;
    int i =0;
    file = fopen(path,"rb");
    if(NULL == file)
    {
        ROS_INFO("open sensor update file failed");
        return -1;
    }
    signal(SIGALRM, send_file_over_time);
    alarm(SENSOR_UPGRADE_OVER_TIME);
    while(!feof(file))
    { 
        ROS_INFO("send sensor upgrade file count:%d",i++);
        len = fread(buffer,1,SENSOR_UPGRADE_FILE_FRAME_LEN,file);
        ROS_INFO("sensor upgrade_file len:%d",len);
	    send_upgrade_file_frame(buffer,len);
		bzero(buffer,SENSOR_UPGRADE_FILE_FRAME_LEN);  
		usleep(SENSOR_SLEEP_TIME);
		handle_receive_data(&sensor_sys); 
		ROS_INFO("sensor upgrade_recv_rlt:%d",sensor_info.upgrade_recv_rlt);
		if(sensor_info.upgrade_recv_rlt != 0x00)
		{
            //if send fail then resend pre frame
            ROS_INFO("sensor upgrade_recv_rlt fail");
			fseek(file,-SENSOR_UPGRADE_FILE_FRAME_LEN,SEEK_CUR);
		}
		if(1 == sensor_over_time_flag){
            ROS_INFO("send sensor upgrade file over time!!");
            fclose(file);
            sensor_over_time_flag = 0;
            return -4;
        }
        sensor_info.upgrade_recv_rlt =-1;
    }
    alarm(0);
	fclose(file);
    ROS_INFO("send sensor upgrade file over");
	return 0;
}

static int send_end_upgrade(void)
{
    unsigned char data[7] = {0};
	data[0] = 0x5A;
	data[1] = 0x07;
    data[2] = 0x0F;
	data[3] = 0x02;
	data[4] = 0;
	data[5] = data[0]+data[1]+data[2]+data[3]+data[4];
	data[6] = 0xA5;
	send_serial(data,&sensor_sys);
	usleep(SENSOR_END_UPGRADE_SLEEP_TIME);
	handle_receive_data(&sensor_sys);
	return 0;
}

int sensor_upgrade_one_count(char * path,char * md5char)
{
    int rlt = 0;
	rlt = send_ready_upgrade(path, md5char);
    ROS_INFO("sensor upgrade_ready_rlt:%d",rlt);
	if((rlt >= 0) && (0x00 == sensor_info.upgrade_ready_rlt))
	{
        rlt = send_upgrade_file(path);
        if(rlt >= 0){
           send_end_upgrade();
           if(0x00 == sensor_info.upgrade_end_rlt)
		   {
               rlt = 0;
		   }
		   else if(0x01 == sensor_info.upgrade_end_rlt)
		   {
               rlt = -5;
		   }
           else
           {
               rlt = -7;
           }
		}
	}
	else if(sensor_info.upgrade_ready_rlt >0)
	{
	    if(0x01 == sensor_info.upgrade_ready_rlt)
        {
           rlt = -2;
	    }
		else if(0x02 == sensor_info.upgrade_ready_rlt)
		{
           rlt = -3;
		}
		else
		{
           rlt = -8;
		}
	}
    else if(-1 == sensor_info.upgrade_ready_rlt)
    {
        rlt = -6;
    }
    ROS_INFO("sensor_upgrade_rlt:%d",rlt); 
	return rlt;
}

/*
 *return -1:file open failed;
 *       -2:sensor mcu memory is not enough
 *       -3:sensor mcu memory execute other job
 *       -4:send file over time 
 *       -5:sensor mcu firmware check error
 *       -6:nv have not ready frame
 *       -7:nv have not end frame
 */
int  sensor_upgrade(char * path,char * md5char)
{
    int upgrade_count = 0;
    int rlt = -1;

    while(upgrade_count < 3)
    {
        rlt = sensor_upgrade_one_count(path,md5char);
        if(rlt < 0)
        {
            upgrade_count++;
        }
        else
        {
            ROS_INFO("sensor upgrade success");
            break;
        }
     }
     return rlt;    
}

void set_safe_distance(double safe_distance)
{
	if(0.0 > safe_distance)
	{
		ROS_INFO("sensor safe_distance can not be negative!");
		return;
	}
	unsigned char data[15]={0};
	int i = 0;
	
	data[0] = 0x5A;
	data[1] = 0x0F;
	data[2] = 0x01;
	for(i=0; i<10 ;i++)
	{
		data[3+i] = (unsigned char)(safe_distance*LEN_M_TO_CM);
    }
	
	for(i=0; i<13; i++)
	{
		data[13] += data[i];
	}
	data[14] = 0xA5;
	
	send_serial(data,&sensor_sys);
    usleep(SENSOR_SLEEP_TIME);
	handle_receive_data(&sensor_sys);
}

void get_safe_distance(void)
{
	unsigned char data[6]={0};
	
	data[0] = 0x5A;
	data[1] = 0x06;
	data[2] = 0x02;
	data[3] = 0;
	data[4] = data[0]+data[1]+data[2]+data[3];
	data[5] = 0xA5;
	send_serial(data,&sensor_sys);
    usleep(SENSOR_SLEEP_TIME);
	handle_receive_data(&sensor_sys);
}

static void get_sensor_send_frame()
{
	unsigned char data[6]={0};
	
	data[0] = 0x5A;
    data[1] = 0x06;
	data[2] = 0x03;
	data[3] = 01;
	data[4] = data[0]+data[1]+data[2]+data[3];
	data[5] = 0xA5;

	send_serial(data,&sensor_sys);
    ROS_INFO("get_sensor_data");
}

void set_function_cali(int function_cali_cmd, int function_cali_param)
{
	unsigned char data[7]={0};

	if(function_cali_cmd < 0)
	{
		function_cali_cmd = 0;
	}
	if(function_cali_param < 0)
	{
        function_cali_param = 0;
    }
	
	data[0] = 0x5A;
	data[1] = 0x07;
    data[2] = 0x0D;
    data[3] = (unsigned char)function_cali_cmd;
    data[4] = (unsigned char)function_cali_param;
    data[5] = data[0]+data[1]+data[2]+data[3]+data[4];
    data[6] = 0xA5;
    send_serial(data,&sensor_sys);
    usleep(SENSOR_SLEEP_TIME);
	handle_receive_data(&sensor_sys);
}

static int check_com_rssi(int send_num,sensor_sys_t *sys)
{
    if(0 == send_num)
    {
        if(sys->rec_num >= 8)
        {
            sys->com_rssi = 4;
        }
        else if(sys->rec_num >= 6)
        {
            sys->com_rssi = 3;
        }
        else if(sys->rec_num >= 4)
        {
            sys->com_rssi = 2;
        }
        else if(sys->rec_num >= 1)
        {
            sys->com_rssi = 1;
        }
        else
        {
            sys->com_rssi = 0;
        }
        
        sys->rec_num = 0;
    }
    return 0;
}

void *sensor_thread_start(void *)
{
    int send_num = 0;
	static int flag = 0;
    sensor_sys.com_state = COM_OPENING;
	ros::NodeHandle nh;

    update_system_state(&sensor_sys);
    double temp_estop_limit = 0;
    //if((sensor_sys.sensor_freq <= 0) || (sensor_sys.sensor_freq > 50.0))
    {
        sensor_sys.sensor_freq = 20;
    }
    ros::Rate loop_rate(sensor_sys.sensor_freq);
    sensor_sys.lasercloud_pub = nh.advertise<sensor_msgs::PointCloud2>("lasercloud", 50, true);
	sensor_sys.sensor_pub = nh.advertise<SensorMsg>("sensor_msg", 2, true);
    hall_pub = nh.advertise<std_msgs::String>("hall_msg",20);

    while(ros::ok()) 
    {  
        ROS_INFO("ros OK!!");
        if(0 == sensor_sys.upgrade_status)
        {
            update_system_state(&sensor_sys);
            handle_receive_data(&sensor_sys);
            if(COM_RUN_OK == sensor_sys.com_state)
            {
                check_com_rssi(send_num,&sensor_sys);
                if(temp_estop_limit != sensor_sys.estop_limit)
                {
                   set_safe_distance(sensor_sys.estop_limit);
                   get_safe_distance();
                   temp_estop_limit = sensor_sys.estop_limit;
                }
                ROS_INFO("com_state OK!!");
                get_sensor_send_frame();
                send_num=(send_num + 1)%10;
            }
        }
        else if(1 == sensor_sys.upgrade_status)
        {
             if(flag == 0)
             {
                flag = 1;
                sensor_sys.upgrade_result = sensor_upgrade(sensor_sys.name,sensor_sys.md5);
                sensor_sys.upgrade_status = 0;
                flag = 0;
             }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    close(sensor_sys.com_device);
    sensor_sys.work_normal = 0;
	sensor_sys.com_state = COM_OPENING;
    sensor_sys.com_rssi = 0;
    return 0;
}

int get_sensor_data(system_t *sys)
{
    int i = 0;

    if(NULL == sys)
    {
        ROS_INFO("sys NULL!");
        return -1;
    }

    if(0 == sensor_sys.handle_data_flag)
    {
        sensor_sys.handle_data_flag = 1;
        for(i=0;i < LASER_NUM;i++)
        {
            sys->sensor.laser_len[i] = sensor_sys.laser_len[i];
        }
        for(i=0;i < SONAR_NUM;i++)
        {
            sys->sensor.sonar_len[i] = sensor_sys.sonar_len[i];
        }
        sys->sensor.estop_io_flag = sensor_sys.estop_io_flag;
        sys->sensor.infrared_flag = sensor_sys.infrared_flag;
        for(i=0;i < SENSOR_STATUS_NUM;i++)
        {
            sys->sensor.status[i] = sensor_sys.status[i];
        }
		sys->sensor.estop_fb_limit = sensor_sys.estop_fb_limit;
        sys->sensor.work_normal = sensor_sys.work_normal;
        sys->sensor.com_rssi = sensor_sys.com_rssi;
        for(i = 0;i<VERSION_LEN;i++)
        {
            sys->sensor.version[i] = sensor_sys.software_version[i];
        }
        sensor_sys.handle_data_flag = 0;
    }
    return 0;
}

int set_sensor_upgrade(char *str,char *md5)
{
    if((NULL == str) || (NULL == md5))
	{
	    return -1;
	}
    if(0 == sensor_sys.upgrade_status)
	{
		memcpy(sensor_sys.name,str,strlen(str));
		memcpy(sensor_sys.md5,md5,MD5_SIZE);
		sensor_sys.upgrade_status = 1;//info sensors.cpp to upgrade sensor
		return 0;
	}
    return 1;
}

int get_sensor_upgrade_status(void)
{
    int i = sensor_sys.upgrade_status;
    return i;
}

int get_sensor_upgrade_result(void)
{
    int i = sensor_sys.upgrade_result;
    return i;
}

int set_sensors_cmd(system_t *sys)
{
    if(NULL == sys)
    {
        ROS_INFO("sys NULL!");
        return -1;
    }
    if(sensor_sys.estop_limit != sys->sensor.estop_limit)
    {
        sensor_sys.estop_limit = sys->sensor.estop_limit;
    }
    
    return 0;
}

