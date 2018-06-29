#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"

#include <sstream>
#include <math.h>
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
#include "../include/starline/move.h"

static move_sys_t move_sys;
static move_info_t move_info;
static int move_over_time_flag = 0;
static int last_unread_bytes = 0;
static unsigned char recv_buf_last[BUF_LEN] = {0};


//20170706,Zero
unsigned char baseStateData[7]={0};
static void loadMotorCMD(uint8_t cmd);
unsigned char loadFlag = 0;
unsigned char loadCMD = 0 ;

static void handle_rev_frame(move_sys_t *sys,unsigned char * frame_buf)
{
    int frame_len = 0;
	int i = 0;
    int j = 0;
    float tmp = 0.0;
	unsigned char check_data = 0;
	frame_len = frame_buf[1];

    for(i=0;i<frame_len-2;i++)
    {
        check_data += frame_buf[i];
    }

	if(check_data != frame_buf[frame_len-2] || 0xA5 != frame_buf[frame_len -1])
	{
        ROS_DEBUG("receive frame check error");
		return;
	}
/*for(i =0;i<frame_len;i++){
ROS_DEBUG("move receive:%02x",frame_buf[i]);
}*/
	if(2 != sys->work_normal)
    {
         sys->work_normal = 2;
    }
	if(0 == sys->handle_data_flag)
    {
         sys->handle_data_flag = 1;
		 move_info.recv_type = frame_buf[2];

		 switch (move_info.recv_type)
		 {
             case 0x60:
                move_info.move_open_station_ack = frame_buf[3];
                break;
                                 
			 case 0x61:
				sys->move_open_station = frame_buf[3];
                break;
								 
			 case 0x62:
				move_info.high_limit_ack = frame_buf[3];
				move_info.low_limit_ack = frame_buf[4];
                break;
								 
			 case 0x63:
                sys->high_limit = frame_buf[3]*LEN_M_TO_MM;
				sys->low_limit = frame_buf[4]*LEN_M_TO_MM;
				break;

			 case 0x64:
                move_info.sensor_state_ack = frame_buf[3];
                break;

			 case 0x65:
                sys->move_sensor_state = frame_buf[3];
				break;

			 case 0x66:
			 	for(i=0;i<BASE_MOTOR_NUM;i++)
                {
                    move_info.motor_status_ack[i] = frame_buf[3+i];
                }
				break;

			 case 0x67:
			 	//0->left,1->right
			 	for(i=0;i<BASE_MOTOR_NUM;i++)
                {
                    sys->motor_status[i] = frame_buf[3+i];
                }
				break;

			 case 0x68:
			 	*(char*)&tmp = frame_buf[6];
                *((char*)&tmp+1) = frame_buf[5];
                *((char*)&tmp+2) = frame_buf[4];
                *((char*)&tmp+3) = frame_buf[3];
                sys->odom.x = tmp/1000.0;
                *(char*)&tmp = frame_buf[10];
                *((char*)&tmp+1) = frame_buf[9];
                *((char*)&tmp+2) = frame_buf[8];
                *((char*)&tmp+3) = frame_buf[7];
                sys->odom.y = tmp/1000.0;
                *(char*)&tmp = frame_buf[14];
                *((char*)&tmp+1) = frame_buf[13];
                *((char*)&tmp+2) = frame_buf[12];
                *((char*)&tmp+3) = frame_buf[11];
                sys->odom.th = tmp;
				
				sys->fb_vel.vx = ((double)((signed short)
					((frame_buf[15]<<8)|(frame_buf[16]))))/1000.0;
                sys->fb_vel.vth = ((double)((signed short)
					((frame_buf[17]<<8)|(frame_buf[18]))))/1000.0;
				//0->left,1->right
				
				for(i=0;i<BASE_MOTOR_NUM;i++)
                {
                    sys->motor_status[i] = frame_buf[19+i];
                }
				sys->move_status = frame_buf[21];
                sys->estop_sensor_flag = frame_buf[22];
                for(i=0;i<BASE_LASER_NUM;i++)
                {
                    sys->laser[i] = ((double)frame_buf[23+i])/1000.0;
                }
				sys->move_sensor_state = frame_buf[26];

                //20170712
                baseStateData[0] = frame_buf[19];//left
                baseStateData[1] = frame_buf[20];//right
                baseStateData[2] = frame_buf[21];//status
                baseStateData[3] = frame_buf[22];//collision state
				baseStateData[4] = frame_buf[26];//load motor
				baseStateData[5] = frame_buf[25];//load state
				baseStateData[6] = frame_buf[23];//load switchs
				

				break;
			case 0x69:
                for(j=0; j<HANDSPIKE_STATUS_NUM; j++)
				{
                    sys->handspike_status[j] = frame_buf[3+j]; 
                    baseStateData[4+j] = frame_buf[3+j]; // 4 -- loadmotor  5 -- working state  6 -- swutchs state
//					ROS_INFO("baseStateData[%d] = %x" ,j,baseStateData[4+j]);
				}
				break;
           /*
           case 0x6A:
                for(j=0; j<HANDSPIKE_STATUS_NUM; j++)
				{
                    sys->handspike_status[j] = frame_buf[3+j]; 
				}
				break;
           case 0x6B:
                for(j=0; j<HANDSPIKE_STATUS_NUM; j++)
				{
                    sys->handspike_status[j] = frame_buf[3+j]; 
				}
				break;				
			*/
            case 0x6E:
                for(j=0; j<MOVE_HARDWARE_VER_LEN; j++)
				{
                    sys->hardware_version[j] = frame_buf[3+j]; 
				}
				for(j=0; j<MOVE_SOFTWARE_VER_LEN; j++)
				{
                    sys->software_version[j] = frame_buf[7+j];
				}
				break;
				
			case 0x6F:
				move_info.upgrade_type = (int)frame_buf[3];
				switch(move_info.upgrade_type)
				{
                      case 0x00:
                         move_info.upgrade_ready_rlt = (int)frame_buf[4];
                         ROS_DEBUG("move_info.upgrade_ready_rlt:%02x",move_info.upgrade_ready_rlt);
						 break;
										  
					  case 0x01:                    
						 move_info.upgrade_recv_rlt = (int)frame_buf[4];
						 break;

					  case 0x02:
						 move_info.upgrade_end_rlt = (int)frame_buf[4];
                         ROS_DEBUG("move_info.upgrade_end_rlt:%02x",move_info.upgrade_end_rlt);
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

static int handle_receive_data(move_sys_t *sys)
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
	
    if(NULL == sys)
    {
        ROS_DEBUG("sys NULL!");
        return -1;
    }
    if(COM_RUN_OK != sys->com_state && COM_CHECK_VERSION != sys->com_state)
    {
        ROS_DEBUG("move_handle_receive_data: com_state != COM_RUN_OK && COM_CHECK_VERSION");
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
        //ROS_DEBUG("move nread:%d",nread);
        //ROS_DEBUG("move last_unread_bytes:%d",last_unread_bytes);
        
        memcpy(recv_buf_complete+last_unread_bytes,recv_buf,nread);
        data_Len = last_unread_bytes + nread;
        last_unread_bytes = 0;

        while(i<data_Len)
        {
            if(0x5A == recv_buf_complete [i])
            {
                
                 frame_len = recv_buf_complete[i+1]; 
                 if(i+frame_len <= data_Len)
                 {
                      if(0xA5 == recv_buf_complete[i+frame_len-1])
                      {
                           for(j=0;j<frame_len;j++)
                           {
                               recv_buf_temp[j] = recv_buf_complete[i+j];
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

static int send_serial(unsigned char *send_buf,move_sys_t *sys)
{
    int len = 0;

	int send_buf_len = 0;

    if((NULL == sys) || (NULL == send_buf))
    {
        ROS_DEBUG("sys or send_buf NULL!");
        return -1;
    }
    
    if(COM_RUN_OK != sys->com_state && COM_CHECK_VERSION != sys->com_state)
    {
        ROS_DEBUG("move_send_serial: com_state != COM_RUN_OK && COM_CHECK_VERSION");
        return 0;
    }

    send_buf_len = send_buf[1];
	if(send_buf_len <= 0)
	{
        ROS_DEBUG("send_buf len: %d small 0!",send_buf_len);
        return -1;
	}
/*for(int i =0;i<send_buf_len;i++){
       ROS_DEBUG("move send_buf :%02x",send_buf[i]);
    }*/
    len = write(sys->com_device,send_buf,send_buf_len);
    if (len == send_buf_len)
    {
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

void get_move_version(void)
{
	unsigned char data[5]={0};
	
	data[0] = 0x5A;
	data[1] = 0x05;
	data[2] = 0x6E;
	data[3] = data[0]+data[1]+data[2];
	data[4] = 0xA5;
	send_serial(data,&move_sys);
    usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
}

static bool check_version(void)
{
    int i = 0;
    for(i=0; i<MOVE_HARDWARE_VER_LEN; i++)
	{
        if('\0' == move_sys.hardware_version[i])
        {
             return false;
        }            
	}
	for(i=0; i<MOVE_SOFTWARE_VER_LEN; i++)
	{
         if('\0' == move_sys.software_version[i]) 
         {
			 return false;
         }
    } 
    return true;		
}

static void move_info_init(void)
{	
	char com_device_path[]="/dev/ros/movebase";//"/dev/ttyUSB0";
   
    memcpy(move_sys.dev,com_device_path,sizeof(com_device_path));
    
    move_sys.com_rssi = 0;
    move_sys.cmd_vel.vx = 0.0;
    move_sys.cmd_vel.vth = 0.0;
	move_sys.move_status = 0x06;
	move_sys.work_normal = 0;

    move_info.move_open_station_ack =-1;
	move_info.upgrade_ready_rlt = -1;
	move_info.upgrade_recv_rlt = -1;
	move_info.upgrade_end_rlt = -1;
   
}

static void update_system_state(move_sys_t *sys)
{
    static int last_file_flag = 0;
    struct stat file_info;
    int i = 0;

    if(NULL == sys)
    {
        ROS_DEBUG("sys NULL!");
        return;
    }
    
    switch(sys->com_state)
    {
        case COM_OPENING:
			sys->work_normal = 0;
            move_info_init();  
			sys->move_rssi = 0;
            i = stat(sys->dev,&file_info);
            if(-1 == i)
            {
                if(-1 != last_file_flag)
                {
                    ROS_DEBUG("move com device does not exist\n");
                }
                last_file_flag = i;
                return;
            }
            last_file_flag = i;
            sys->com_device = open_com_device(sys->dev);
            if(-1 != sys->com_device)
            {
                sys->com_state = COM_CHECK_VERSION;
                sys->com_rssi = 4;
                sys->rec_num = 8;
				sys->work_normal = 1;
                ROS_DEBUG("open move com success!");
            }
            else
            {
                ROS_DEBUG("open move com device failed");
                return;
            }
            set_speed(sys->com_device,115200);
            set_parity(sys->com_device,8,1,'N');
             
            break;
            
        case COM_CHECK_VERSION:
            get_move_version();
            if(check_version())
            {
                 sys->com_state = COM_RUN_OK;
                 //close io stop
                 set_sensor_function(0x0A);
            }
            break;

        case COM_RUN_OK:
            //ROS_DEBUG("COM_RUN_OK!!");
            break;
            
        case COM_CLOSING:
            close(sys->com_device);
            sys->com_state = COM_OPENING;
            sys->com_rssi = 0;
			sys->move_rssi = 0;
			sys->work_normal = 0;
            ROS_DEBUG("close com device:%d\n",sys->com_device);
            break;    
        default:
            break;
    }

    return;
}

static int check_move_rssi(int send_num,move_sys_t *sys)
{
    if(NULL == sys)
    {
        ROS_DEBUG("sys NULL!");
        return -1;
    }
    
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

static long get_upgrade_size(char* path)
{
	FILE *file;
	
	long len = 0;
	
	file = fopen(path,"rb");
	if(NULL == file)
	{
		ROS_DEBUG("get move upgrade size failed");
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
	data[2] = 0x6F;
	data[3] = 0x00;
											 
	for(i=0; i<16; i++)
	{
		data[i+4] = md5char[i];
	}
    
    //big end or small end convert socket frame
    filesize.lnum = htonl(get_upgrade_size(path));
    ROS_DEBUG("get_upgrade_size:%ld",filesize.lnum);
	if(0 == filesize.lnum){
       ROS_DEBUG("move upgrade file is 0\n");   
	   return -1;
	}

	for(i=0; i<4; i++)
	{
        data[i+20] = filesize.cnum[i];
        ROS_DEBUG("move upgrade file size:%c",filesize.cnum[i]);  
	}
    
	for(i=0; i<24; i++)
	{
		data[24] += data[i];
	}
	data[25] = 0xA5;  
	send_serial(data,&move_sys);
	usleep(MOVE_READY_UPGRADE_SLEEP_TIME);
	handle_receive_data(&move_sys);
	return 0;
}

static int send_upgrade_file_frame(char* buffer,int len)
{
     unsigned char data[MOVE_UPGRADE_FILE_FRAME_LEN+6]={0};
	 int i = 0;
	 
	 data[0] = 0x5A; 
	 data[1] = len+6;
	 data[2] = 0x6F;
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
	 send_serial(data,&move_sys); 
	 return 0;
}

static void send_file_over_time(int sig)
{    
    move_over_time_flag =1;
}

static int send_upgrade_file(char* path)
{	
	char buffer[MOVE_UPGRADE_FILE_FRAME_LEN]={0};
	FILE *file;
    int len = 0;
    int i =0;

    file = fopen(path,"rb");
    if(NULL == file)
    {
        ROS_DEBUG("open move update file failed");
        return -1;
    }
    signal(SIGALRM, send_file_over_time);
    alarm(MOVE_UPGRADE_OVER_TIME);
    while(!feof(file))
    { 
        ROS_DEBUG("send move upgrade file count:%d",i++);
        len = fread(buffer,1,MOVE_UPGRADE_FILE_FRAME_LEN,file);
        ROS_DEBUG("move upgrade_file len:%d",len);
        
	    send_upgrade_file_frame(buffer,len);
		
		usleep(MOVE_SLEEP_TIME);
		handle_receive_data(&move_sys); 
		ROS_DEBUG("move upgrade_recv_rlt:%d",move_info.upgrade_recv_rlt);
		if(move_info.upgrade_recv_rlt != 0x00)
		{
            //if send fail then resend pre frame
            ROS_DEBUG("move upgrade_recv_rlt fail");
			fseek(file,-MOVE_UPGRADE_FILE_FRAME_LEN,SEEK_CUR);
		}
		if(1 == move_over_time_flag){
            ROS_DEBUG("send move upgrade file over time!!");
            fclose(file);
            move_over_time_flag = 0;
            return -4;
        }
        bzero(buffer,MOVE_UPGRADE_FILE_FRAME_LEN);  
        move_info.upgrade_recv_rlt =-1;
    }
    alarm(0);
	fclose(file);
    ROS_DEBUG("send upgrade file over");
	return 0;
}

static int send_end_upgrade(void)
{
    unsigned char data[7] = {0};
	data[0] = 0x5A;
	data[1] = 0x07;
    data[2] = 0x6F;
	data[3] = 0x02;
	data[4] = 0;
	data[5] = data[0]+data[1]+data[2]+data[3]+data[4];
	data[6] = 0xA5;
	send_serial(data,&move_sys);
	usleep(MOVE_END_UPGRADE_SLEEP_TIME);
	handle_receive_data(&move_sys);
	return 0;
}

static int move_upgrade_one_count(char * path,char * md5char)
{
    int rlt = 0;
	rlt = send_ready_upgrade(path, md5char);
    ROS_DEBUG("move upgrade_ready_rlt:%d",rlt);
	if((rlt >= 0) && (0x00 == move_info.upgrade_ready_rlt))
	{
        rlt = send_upgrade_file(path);
        if(rlt >= 0){
           send_end_upgrade();
           if(0x00 == move_info.upgrade_end_rlt)
		   {
               rlt = 0;
		   }
		   else if(0x01 == move_info.upgrade_end_rlt)
		   {
               rlt = -5;
		   }
           else if(0x02 == move_info.upgrade_end_rlt)
		   {
               rlt = -9;
		   }
		   else if(0x04 == move_info.upgrade_end_rlt)
		   {
               rlt = -10;
		   }
		   else if(0x08 == move_info.upgrade_end_rlt)
		   {
               rlt = -11;
		   }
           else
           {
               rlt = -12;
           }
		}
	}
	else if(move_info.upgrade_ready_rlt >0)
	{
	    if(0x01 == move_info.upgrade_ready_rlt)
        {
           rlt = -2;
	    }
		else if(0x02 == move_info.upgrade_ready_rlt)
		{
           rlt = -3;
		}
        else
		{
           rlt = -8;
		}
	}
    else if(-1 == move_info.upgrade_ready_rlt)
    {
        rlt = -6;
    }
    ROS_DEBUG("move_upgrade_rlt:%d",rlt); 
	return rlt;
}

/*
 *return -1:file open failed;
 *       -2:move mcu memory is not enough
 *       -3:move mcu memory execute other job
 *       -4:send move file over time 
 *       -5:move mcu firmware check error
 *       -6:nv have not ready frame
 *       -7:nv have not end frame
 *       -8:ready frame erase flash fail
 *       -9:data flash erase fail
 */
int move_upgrade(char * path,char * md5char)
{
    int upgrade_count = 0;

    int rlt = -1;
    while(upgrade_count < 3)
    {
        rlt = move_upgrade_one_count(path,md5char);
        if(rlt < 0)
        {
            upgrade_count++;
        }
        else
        {
            ROS_DEBUG("move upgrade success");
            break;
        }
     }
     ROS_DEBUG("move upgrade_count:%d",upgrade_count);
     return rlt;    
}

int clear_open_signal(void)
{
    int rlt =-1;
	unsigned char data[5]={0};
	data[0] = 0x5A;
	data[1] = 0x05;
	data[2] = 0x60;
	data[3] = data[0]+data[1]+data[2];
	data[4] = 0xA5;
	send_serial(data,&move_sys);
	usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);

	if(0 == move_info.move_open_station_ack)
	{
        rlt = 0;
		move_info.move_open_station_ack = -1;
	}
	return rlt;
}

void get_open_sigal(void)
{
	unsigned char data[5]={0};
	data[0] = 0x5A;
	data[1] = 0x05;
	data[2] = 0x61;
	data[3] = data[0]+data[1]+data[2];
	data[4] = 0xA5;
	send_serial(data,&move_sys);
	usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
}

int set_base_limit(double high_limit,double low_limit)
{
    int rlt = -1;
	unsigned char data[7]={0};
	data[0] = 0x5A;
	data[1] = 0x07;
	data[2] = 0x62;
	data[3] = (unsigned char)high_limit*LEN_M_TO_MM;
	data[4] = (unsigned char)low_limit*LEN_M_TO_MM;
	data[5] = data[0]+data[1]+data[2]+data[3]+data[4];
	data[6] = 0xA5;
	send_serial(data,&move_sys);
	usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
    if(move_info.high_limit_ack == high_limit*LEN_M_TO_MM
		&& move_info.low_limit_ack == low_limit*LEN_M_TO_MM)
    {
          rlt = 0;
    }
	return rlt;
}

void get_base_limit(void)
{
	unsigned char data[5]={0};
	data[0] = 0x5A;
	data[1] = 0x05;
	data[2] = 0x63;
	data[3] = data[0]+data[1]+data[2];
	data[4] = 0xA5;
	send_serial(data,&move_sys);
	usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
}

int set_sensor_function(unsigned char senor_state)
{
    int rlt = -1;
	unsigned char data[6]={0};
	data[0] = 0x5A;
	data[1] = 0x06;
	data[2] = 0x64;
	data[3] = senor_state;
	data[4] = data[0]+data[1]+data[2]+data[3];
	data[5] = 0xA5;
	send_serial(data,&move_sys);
    usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
    if(move_info.sensor_state_ack == senor_state)
    {
        rlt = 0; 
    }
	return rlt;
}

void get_sensor_function(void)
{
	unsigned char data[5]={0};
	data[0] = 0x5A;
	data[1] = 0x05;
	data[2] = 0x65;
	data[3] = data[0]+data[1]+data[2];
	data[4] = 0xA5;
	send_serial(data,&move_sys);
	usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
}

int clear_error_state(void)
{
    int rlt = 0;
	unsigned char data[5]={0};
	data[0] = 0x5A;
	data[1] = 0x05;
	data[2] = 0x66;
	data[3] = data[0]+data[1]+data[2];
	data[4] = 0xA5;
	send_serial(data,&move_sys);
    usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
    //mcu do not support clear
	return rlt;
}

void get_error_state(void)
{
	unsigned char data[5]={0};
	data[0] = 0x5A;
	data[1] = 0x05;
	data[2] = 0x67;
	data[3] = data[0]+data[1]+data[2];
	data[4] = 0xA5;
	send_serial(data,&move_sys);
	usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
}

static void move_send_frame(move_sys_t *sys)
{
    short int stmp = 0;
    unsigned char data[9]={0};
	
	data[0] = 0x5A;
    data[1] = 0x09;
	data[2] = 0x68;
	stmp = (short int)(sys->cmd_vel.vx*1000.0);
	data[3] = (stmp & 0xff00)>>8;
    data[4] = (stmp & 0x0ff);
    stmp = (short int)(sys->cmd_vel.vth*1000.0);
    ROS_DEBUG("vth,stmp is :%d\n",stmp);
    data[5] = (stmp & 0xff00)>>8;
    data[6] = (stmp & 0x0ff);
	data[7] = data[0]+data[1]+data[2]+data[3]+data[4]+data[5]+data[6];
	data[8] = 0xA5;

	send_serial(data,&move_sys);
}

void handspike_power_send_frame(void)
{
    unsigned char data[6]={0};
	data[0] = 0x5A;
    data[1] = 0x06;
	data[2] = 0x69;
	data[3] = 0x03;
	data[4] = data[0]+data[1]+data[2]+data[3];
    data[5] = 0xA5;

	send_serial(data,&move_sys);
    usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
}

void handspike_lift_send_frame(void)
{
    unsigned char data[6]={0};
	data[0] = 0x5A;
    data[1] = 0x06;
	data[2] = 0x69;
	data[3] = 0x01;
	data[4] = data[0]+data[1]+data[2]+data[3];
    data[5] = 0xA5;

	send_serial(data,&move_sys);
    usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
}

void handspike_down_send_frame(void)
{
    unsigned char data[6]={0};
	data[0] = 0x5A;
    data[1] = 0x06;
	data[2] = 0x69;
	data[3] = 0x02;
	data[4] = data[0]+data[1]+data[2]+data[3];
    data[5] = 0xA5;

  send_serial(data,&move_sys);
    usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
}

void handspike_stop_send_frame(void)
{
    unsigned char data[6]={0};
	data[0] = 0x5A;
    data[1] = 0x06;
	data[2] = 0x69;
	data[3] = 0x00;
	data[4] = data[0]+data[1]+data[2]+data[3];
    data[5] = 0xA5;

	send_serial(data,&move_sys);
    usleep(MOVE_SLEEP_TIME);
	handle_receive_data(&move_sys);
}



void *movebase_thread_start(void *)
{
    int send_num = 0;  
	static int flag = 0;
    move_sys.com_state = COM_OPENING;
    update_system_state(&move_sys);
    if((move_sys.move_freq <= 0) || (move_sys.move_freq >20))
    {
        move_sys.move_freq = 20;
    }
    ros::Rate loop_rate(move_sys.move_freq);
    ROS_DEBUG("movebase thread is running!");
    
    while(ros::ok()) 
    {  
	
		if(1 == loadFlag )
		{
			loadMotorCMD(loadCMD);
			loadFlag = 0;
		}
        else if(0 == move_sys.upgrade_status)
        {
            update_system_state(&move_sys);
            handle_receive_data(&move_sys);

            if(COM_RUN_OK == move_sys.com_state)
            {
		        check_move_rssi(send_num,&move_sys); 
                move_send_frame(&move_sys);
                send_num=(send_num + 1)%10;
            }
        }
		else if(1 == move_sys.upgrade_status)
		{
             if(flag == 0)
             {
                 flag = 1;
                 move_sys.upgrade_result = move_upgrade(move_sys.name,move_sys.md5);
			     move_sys.upgrade_status = 0;
				 flag = 0;
             }		 
		}

        ros::spinOnce();
        loop_rate.sleep(); 
    }
    close(move_sys.com_device);
	move_sys.com_state = COM_OPENING;
    move_sys.com_rssi = 0;
    move_sys.work_normal = 0;
    return 0; 
}

void set_movebase_cmd_vel(vel_t vel)
{  
    move_sys.cmd_vel.vx = vel.vx;
    move_sys.cmd_vel.vth = vel.vth;
    move_sys.cmd_vel.vy = 0.0;
    return;
}

move_sys_t *get_movebase_info(void)
{
    if(0 == move_sys.handle_data_flag)
    {
        return &move_sys;
    }
    else
    {
        return NULL;
    }
}

int set_base_high_limit(double limit)
{
    if(0 == move_sys.handle_data_flag)
    {
        move_sys.handle_data_flag = 1;
    	move_sys.high_limit = limit;
		move_sys.handle_data_flag = 0;
		return 0;
    }
	return -1;
}

int set_base_low_limit(double limit)
{
    if(0 == move_sys.handle_data_flag)
    {
        move_sys.handle_data_flag = 1;
    	move_sys.low_limit = limit;
		move_sys.handle_data_flag = 0;
		return 0;
    }
	return -1;
}

int set_movebase_upgrade(char *str,char *md5)
{
    if((NULL == str) || (NULL == md5))
	{
	    return -1;
	}
    if(0 == move_sys.upgrade_status)
	{
		memcpy(move_sys.name,str,strlen(str));
		memcpy(move_sys.md5,md5,MD5_SIZE);
		move_sys.upgrade_status = 1;
		return 0;
	}
    return 1;
}

int get_movebase_upgrade_result(void)
{
    int i = move_sys.upgrade_result;
    return i;
}

int get_movebase_upgrade_status(void)
{
    int i = move_sys.upgrade_status;
    return i;
}


//20170815,Zero
static void loadMotorCMD(uint8_t cmd)
{
  uint8_t txData[6];
  txData[0] = 0x5A;
  txData[1] = 0x06;
  txData[2] = 0x69;
  txData[3] = cmd;
  txData[4] = 0x00;
  for(int i =0 ; i < 4 ; i++)
  {
    txData[4] += txData[i] ;
  }
  txData[5] = 0xA5 ;
	
  for(int i = 0 ; i< 6; i++)
	ROS_INFO("txData[%d] = %x" ,i,txData[i]);
  send_serial(txData,&move_sys);
  usleep(MOVE_SLEEP_TIME);
  handle_receive_data(&move_sys);

}
