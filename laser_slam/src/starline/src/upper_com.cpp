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
#include "../include/starline/config.h"
#include "../include/starline/handle_command.h"
#include "../include/starline/system.h"


#define LISTEN_PORT (30102)
#define MAX_LISTEN_NUM (5)
#define CHECK_BEAR_TIMES (1500)

static upper_com_sys_t upper_com_sys;

static void update_upper_state(upper_com_sys_t *sys)
{
	struct sockaddr_in srv_addr;
	switch(sys->socket_status)
	{
		case 1:
			sys->work_normal = 0;
			sys->client_socket = socket(AF_INET, SOCK_STREAM, 0);
			if(sys->client_socket < 0)
			{
				ROS_DEBUG("socket builded failed,:%d!",sys->client_socket);
			}
			else
			{
				sys->socket_status = 2;
			}
			ROS_DEBUG("socket builded ok,:%d!",sys->client_socket);
			break;
		case 2:
            ROS_DEBUG("upper server ip is :%x",sys->server_ip);
			if(0 == sys->server_ip)
			{
			    return;
			}
			memset((void *)&srv_addr, 0, sizeof(srv_addr));
			srv_addr.sin_family = AF_INET;
			srv_addr.sin_port = htons(LISTEN_PORT);
			//srv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
			//srv_addr.sin_addr.s_addr = inet_addr("192.168.3.6");
			srv_addr.sin_addr.s_addr = (unsigned long)sys->server_ip;
            ROS_DEBUG("upper srv_addr.sin_addr.s_addr is :%x",srv_addr.sin_addr.s_addr);
			if(connect(sys->client_socket, (struct sockaddr *)(&srv_addr), sizeof(srv_addr)) < 0)
			{
				ROS_DEBUG("connect failed!");
			}
			else
			{
				sys->socket_status = 5;
				sys->work_normal = 1;
                ROS_DEBUG("socket connected ok!");
			}
			break;
		case 5:
			if(0 != sys->socket_error)
			{
				sys->socket_status = 1;
				sys->socket_error = 0;
				close(sys->client_socket);
				ROS_DEBUG("socket error,close socket!");
			}
			break;
		default:
            ROS_DEBUG("default socket status:%d",sys->socket_status);
			break;
	}
}

static int handle_receive_data(upper_com_sys_t *sys)
{
    int recv_len = 0;
	//int i = 0;
	unsigned char buf[UPPER_COM_RECV_LEN];

	if(5  != sys->socket_status)
	{
	    return -1;
	}
    //ROS_DEBUG("socket recv data\n");
    if(sys->socket_status == 5)
    {
        
		recv_len = recv(sys->client_socket, buf,UPPER_COM_RECV_LEN, MSG_DONTWAIT);
		//ROS_DEBUG("socket recv,recv_len:%d",recv_len);
	    if(recv_len <= 0)
	    {
	        
			/*printf("errno:%d,EINTR:%d,EWOULDBLOCK:%d,EAGAIN:%d\n",
				errno,EINTR,EWOULDBLOCK,EAGAIN);
			perror("perror:");
			printf("strerror:%s\n",strerror(errno));*/
			if(0 == recv_len)
			{
			    ROS_DEBUG("socket 0 == recv error,errno:%d\n",errno);
			    sys->socket_error = 1;
			}
			else if((EINTR != errno) && (EWOULDBLOCK != errno) && (EAGAIN != errno))
			{
			    ROS_DEBUG("socket recv error,errno:%d\n",errno);
			    sys->socket_error = 1;
			}
			return -1;
	    }	
		else
		{
		    if(2 != sys->work_normal)
		    {
		        sys->work_normal = 2;
		    }
			if(0 == sys->handle_upper_com_flag)
			{
			    sys->handle_upper_com_flag = 1;
                
                if((sys->read_num < UPPER_COM_HANDLE_LEN - recv_len) && (recv_len < UPPER_COM_HANDLE_LEN))
                {
					memcpy(sys->upper_com_buf+sys->read_num,buf,recv_len);
				    sys->read_num += recv_len;
			        sys->need_read_flag = 1;
                }
				else
				{
				    ROS_DEBUG("read num < UPPER_COM_HANDLE_LEN\n");
				}
				
				sys->handle_upper_com_flag = 0;
			}
			else
			{
			    ROS_DEBUG("handle upper com flag==1,discard socket data\n");
			}
		}
    }
	return 0;
}

void check_upper_connect(upper_com_sys_t *sys)
{
    static int heart_beat = 1;
	unsigned char data[BUF_LEN]= {0,};

	if(5 != sys->socket_status)
	{
	    ROS_DEBUG("socket status is wrong:%d",sys->socket_status);
	    return;
	}
	
    if(CHECK_BEAR_TIMES-2 == heart_beat%CHECK_BEAR_TIMES)
	{
	    if(0 == sys->upper_beat_flag)
	    {
	        //if has not received heart beat feedback pkg,set socket error to 1,to reconnect socket
            ROS_DEBUG("socket heart beat");
	        sys->socket_error = 1;
	    }
		sys->upper_beat_flag = 0;
	}
	else if(CHECK_BEAR_TIMES-1 == heart_beat%CHECK_BEAR_TIMES)
	{
	    //send heart beat pkg;
	    send_pkg(PKG_SEND_HEART_BEAT,0,0,data);
	}
	heart_beat++;
}
void *upper_com_thread_start(void *)
{
	upper_com_sys.com_rssi = 0;
	upper_com_sys.read_num = 0;
	upper_com_sys.socket_status = 1;
	upper_com_sys.socket_error = 0;
	upper_com_sys.work_normal = 0;
	upper_com_sys.upper_beat_flag = 1;

	if((upper_com_sys.upper_com_freq <= 0) || (upper_com_sys.upper_com_freq >150))
	{
	    upper_com_sys.upper_com_freq = 150;
	}
    ros::Rate loop_rate(upper_com_sys.upper_com_freq);
    
    while(ros::ok()) 
    {  
        update_upper_state(&upper_com_sys);
        
        handle_receive_data(&upper_com_sys);

        check_upper_connect(&upper_com_sys);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    close(upper_com_sys.com_device);
    upper_com_sys.com_rssi = 0;
    upper_com_sys.work_normal = 0;
    return 0;
}

upper_com_sys_t* get_upper_com_system_info(void)
{
    if(0 == upper_com_sys.handle_upper_com_flag)
    {
        return &upper_com_sys;
    }
    else
    {
        return NULL;
    }
}

int send_status_back(unsigned char *buf,int num)
{
    int send_len = 0;

	if(NULL == buf)
	{
	    ROS_DEBUG("buf NULL!");
	    return -1;
	}
	
    if(5 != upper_com_sys.socket_status)
    {
        //ROS_DEBUG("send socket failed!,socket status:%d",upper_com_sys.socket_status);
        return -1;
    }
		
    send_len = send(upper_com_sys.client_socket, buf, num, 0);
	if(send_len < 0)
	{
	    ROS_DEBUG("send socket failed!,send:%d\n",send_len);
		upper_com_sys.socket_error = 1;
		return -1;
	}
	//ROS_DEBUG("send socket send:%d\n",send_len);
	return 0;
}

int  set_upper_server_ip(unsigned int ip)
{
    ROS_DEBUG("set upper server ip:%x",ip);
    upper_com_sys.server_ip = ip;
	return 0;
}

unsigned int get_upper_server_ip(void)
{
    return upper_com_sys.server_ip;
}

void set_upper_beat_flag(int data)
{
    upper_com_sys.upper_beat_flag = data;
	return;
}

int upper_socket_status(void)
{
    if(5 != upper_com_sys.socket_status)
    {
        return -1;
    }
	else
	{
	    return 0;
	}
}
