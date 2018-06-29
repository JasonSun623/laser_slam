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
#include "../include/starline/report.h"
#include "../include/starline/handle_command.h"


#define EVENT_BUFFER_SIZE (100)
#define EVENT_COUNT_NUM (20)

static event_t event_buf[EVENT_BUFFER_SIZE];
static int event_num = 0;

int report_nav_status(unsigned char function,ans_status_t *ans,system_t *sys)
{
    int data = 0;
    if((NULL == ans) || (NULL == sys))
    {
        return -1;
    }

    ans->function = function;
	ans->module = MODULE_NAV;
	switch(function)
	{
	    case 0:
			if(1 == sys->read_system_file_flag)
			{
			    data = 0;
			}
			else
			{
			    data = 1;
			}
			ans->level = LEVEL_ERROR;
			ans->len = 1;
			ans->data[0] = data;
			break;
		case 1:
			if(2 == sys->base.work_normal)
			{
			    data = 0;
			}
			else
			{
			    data = 1;
			}
			ans->level = LEVEL_ERROR;
			ans->len = 1;
			ans->data[0] = data;
			break;
		case 2:
			if(2 == sys->led_power.work_normal)
			{
			    data = 0;
			}
			else
			{
			    data = 1;
			}
			ans->level = LEVEL_ERROR;
			ans->len = 1;
			ans->data[0] = data;
			break;
		case 3:
			if(2 == sys->sensor.work_normal)
			{
			    data = 0;
			}
			else
			{
			    data = 1;
			}
			ans->level = LEVEL_ERROR;
			ans->len = 1;
			ans->data[0] = data;
			break;
		case 4:
			if(2 == sys->upper_work_normal)
			{
			    data = 0;
			}
			else
			{
			    data = 1;
			}
			ans->level = LEVEL_ERROR;
			ans->len = 1;
			ans->data[0] = data;
			break;
		default:
			return -1;
			break;
	}
    return 0;
}

int report_camera_status(unsigned char function,ans_status_t *ans,system_t *sys)
{
    int data = 0;

    if((NULL == ans) || (NULL == sys))
    {
        return -1;
    }

    ans->function = function;
    ans->module = MODULE_CAMERA;
    switch(function)
    {
        case 0:
            data = 0; //default ok
            ans->level = LEVEL_ERROR;
            ans->len = 1;
            ans->data[0] = data;
            break;
        case 1:
            data = 0; //default ok
            ans->level = LEVEL_ERROR;
            ans->len = 1;
            ans->data[0] = data;
            break;
        default:
            return -1;
            break;
    }
    return 0;
}

int report_sensor_status(unsigned char function,ans_status_t *ans,system_t *sys)
{
    int data = 0;
	int i = 0;
    if((NULL == ans) || (NULL == sys))
    {
        return -1;
    }

    ans->function = function;
	ans->module = MODULE_SENSOR;
	switch(function)
	{
	    case 0:
			for(i = 0;i < LASER_NUM;i++)
			{
			    if(sys->sensor.laser_len[i] >= SENSOR_LASER_ERR_LIMIT)
			    {
			        data |= (1<<i);
			    }
			}
			for(i=0;i<SONAR_NUM;i++)
			{
			    if(sys->sensor.sonar_len[i] >= SENSOR_SONAR_ERR_LIMIT)
			    {
			        data |= (1<<(i+LASER_NUM));
			    }
			}
            ROS_DEBUG("report err data:%x",data);
			ans->level = LEVEL_ERROR;
			ans->len = 4;
		    ans->data[0] = *((unsigned char *)&data);
            ans->data[1] = *((unsigned char *)&data+1);
            ans->data[2] = *((unsigned char *)&data+2);
            ans->data[3] = *((unsigned char *)&data+3);
			break;
		case 1:
			for(i = 0;i < LASER_NUM;i++)
			{
			    if(sys->sensor.laser_len[i] < sys->sensor.estop_fb_limit)
			    {
			        data |= (1<<i);
			    }
			}
			for(i=0;i<SONAR_NUM;i++)
			{
			    if(sys->sensor.sonar_len[i] < sys->sensor.estop_fb_limit)
			    {
			        data |= (1<<(i+LASER_NUM));
			    }
			}
            ROS_DEBUG("report data:%x",data);
			ans->level = LEVEL_WARN;
			ans->len = 5;
		    ans->data[0] = *((unsigned char *)&data);
            ans->data[1] = *((unsigned char *)&data+1);
            ans->data[2] = *((unsigned char *)&data+2);
            ans->data[3] = *((unsigned char *)&data+3);
			ans->data[4] = (unsigned char)(sys->sensor.estop_fb_limit*LEN_M_TO_CM);
			break;
		default:
			return -1;
			break;
	}
    return 0;
}

int report_base_status(unsigned char function,ans_status_t *ans,system_t *sys)
{
    int data = 0;
	int i = 0;
    if((NULL == ans) || (NULL == sys))
    {
        return -1;
    }

    ans->function = function;
	ans->module = MODULE_BASE;
	switch(function)
	{
	    case 0:
			data = sys->base.motor_status[0];
			data |= (((int)sys->base.motor_status[1])<<8);
			ans->level = LEVEL_ERROR;
			ans->len = 4;
		    ans->data[0] = *((unsigned char *)&data);
            ans->data[1] = *((unsigned char *)&data+1);
            ans->data[2] = *((unsigned char *)&data+2);
            ans->data[3] = *((unsigned char *)&data+3);
			break;
		case 1:
			ans->level = LEVEL_INFO;
			ans->len = 1;
		    ans->data[0] = (sys->base.move_status)>>3;
            ROS_DEBUG("report base status:%x,%x",sys->base.move_status,ans->data[0]);
			break;
		case 2:
			for(i = 0;i < BASE_LASER_NUM;i++)
			{
			    if(sys->base.laser[i] >= BASE_LASER_ERR_LIMIT)
			    {
			        data |= (1<<(4+i));
			    }
			}
			ans->level = LEVEL_ERROR;
			ans->len = 1;
		    ans->data[0] = data;
			break;
		case 3:
			ans->level = LEVEL_WARN;
			ans->len = 1;
		    ans->data[0] = sys->base.estop_sensor_flag;
            ROS_DEBUG("report base estop:%x,%x",sys->base.estop_sensor_flag,ans->data[0]);
			break;
		default:
			return -1;
			break;
	}
    return 0;
}

int report_power_status(unsigned char function,ans_status_t *ans,system_t *sys)
{
    int data = 0;
    if((NULL == ans) || (NULL == sys))
    {
        return -1;
    }

    ans->function = function;
	ans->module = MODULE_POWER;
	switch(function)
	{
	    case 0:
			if(sys->led_power.power_status1&0x20)
			{
			    data = 1;
			}
            ROS_DEBUG("report power status:%x,%d",sys->led_power.power_status1,data);
			ans->level = LEVEL_WARN;
			ans->len = 2;
		    ans->data[0] = data;
			ans->data[1] = sys->led_power.power_v1;
			ans->data[2] = sys->led_power.power_v2;
			break;
		case 1:
			if(sys->led_power.power_status1&0x10)
			{
			    data = 1;
			}
			ans->level = LEVEL_INFO;
			ans->len = 1;
		    ans->data[0] = data;
			break;
		case 2:
			ans->level = LEVEL_WARN;
			ans->len = 4;
			data = (((int)sys->led_power.err4)<<24)|(((int)sys->led_power.err3)<<16)|(((int)sys->led_power.err2)<<8)|((int)sys->led_power.err1);
		    ans->data[0] = *((unsigned char *)&data);
            ans->data[1] = *((unsigned char *)&data+1);
            ans->data[2] = *((unsigned char *)&data+2);
            ans->data[3] = *((unsigned char *)&data+3);
			break;
		case 3:
            data = ((~sys->led_power.power_switch_status1)>>4)&0x07;
            ROS_DEBUG("report power switch status:%x",data);
			ans->level = LEVEL_ERROR;
			ans->len = 1;
		    ans->data[0] = data;
			break;
		default:
			return -1;
			break;
	}
	return 0;
}

int handle_report_status(ask_status_t ask,ans_status_t *ans,system_t *sys,
           motion_t *motion, env_t *env)
{
    int i = 0;
		
    if((NULL == ans) || (NULL == sys) || (NULL == motion) || (NULL == env))
    {
        return -1;
    }

    if((MODULE_NAV != ask.module_group) &&  (MODULE_PAD != ask.module_group))
    {
        return -1;
    }
		
	switch(ask.module)
	{
	    case MODULE_NAV:
			i = report_nav_status(ask.function,ans,sys);
			if(0 != i)
			{
			    return -1;
			}
			break;
		case MODULE_SENSOR:
			i = report_sensor_status(ask.function,ans,sys);
			if(0 != i)
			{
			    return -1;
			}
			break;
		case MODULE_LED:
			//with power board
			break;
		case MODULE_BASE:
			i = report_base_status(ask.function,ans,sys);
			if(0 != i)
			{
			    return -1;
			}
			break;
		case MODULE_POWER:
			i = report_power_status(ask.function,ans,sys);
			if(0 != i)
			{
			    return -1;
			}
			break;
		case MODULE_CAMERA:
			i = report_camera_status(ask.function,ans,sys);
			if(0 != i)
			{
			    return -1;
			}
			break;
		default:
			    return -1;
			break;
	}
    return 0;
}

int report_event(ans_status_t *ans)
{
    unsigned char str[BUF_LEN]={0,};
	int len = 0;
	int i = 0;
		
    if(NULL == ans)
    {
        return -1;
    }

    str[0] = ans->level;
	str[1] = ans->module;
	str[2] = ans->function;
	str[3] = ans->len;
	for(i=0;i<ans->len;i++)
	{
	    str[4+i] = ans->data[i];
	}
	len = ans->len + 4;
	
	i = send_pkg(PKG_FB_CHECK_SYSTEM,len,0,str);
	
	return i;
}

static int check_same_event_msg(ans_status_t *a_msg,ans_status_t *b_msg)
{
    int i = 0;
    if(a_msg->level == b_msg->level)
	{
	    if(a_msg->module == b_msg->module)
	    {
	        if(a_msg->function == b_msg->function)
	        {
	            for(i=0;i<a_msg->len;i++)
	            {
	                if(a_msg->data[i] != b_msg->data[i])
	                {
	                    break;
	                }
	            }
				if(i == a_msg->len)
				{
				    return 1;
				}
	        }
	    }
	}
	return 0;
}
int set_event_buffer(ans_status_t *ans)
{
    int i = 0;
	int j = 0;
		
    if(event_num >= EVENT_BUFFER_SIZE-1)
    {
        //event buffer is full
        event_buf[EVENT_BUFFER_SIZE-1].ans.level = LEVEL_WARN;
		event_buf[EVENT_BUFFER_SIZE-1].ans.module = MODULE_NAV;
		event_buf[EVENT_BUFFER_SIZE-1].ans.function = 24;
		event_buf[EVENT_BUFFER_SIZE-1].ans.len = 4;
		set_int_buf(event_buf[EVENT_BUFFER_SIZE-1].ans.data,1);
		event_buf[EVENT_BUFFER_SIZE-1].flag = 1;
		return -1;
    }
	else
	{
	    for(i=0;i<EVENT_BUFFER_SIZE-1;i++)
	    {
	        j = check_same_event_msg(&(event_buf[i].ans),ans);
			if((1 == j) && (1 == event_buf[i].flag))
			{
			    return 0;
			}
	    }
		for(i=0;i<EVENT_BUFFER_SIZE-1;i++)
		{
		    if(0 == event_buf[i].flag)
		    {
		        break;
		    }
		}
	    event_buf[i].ans.level = ans->level;
		event_buf[i].ans.module = ans->module;
		event_buf[i].ans.function = ans->function;
		event_buf[i].ans.len = ans->len;
		memcpy(event_buf[i].ans.data,ans->data,ans->len);
		event_buf[i].flag = 1;
		event_num++;
	}
	return 0;
}
void handle_report_event(void)
{
    static int count = 0;
	int i = 0;
	int j = 0;

    count++;
	if(0 == (count%EVENT_COUNT_NUM))
	{
	    i = upper_socket_status();
	    if((event_num>0) && (event_num < EVENT_BUFFER_SIZE) && (0 == i))
	    {
	        for(i=0;i<EVENT_BUFFER_SIZE;i++)
	        {
	            if(1 == event_buf[i].flag)
	            {
	                j = report_event(&(event_buf[i].ans));
					if(0 == j)
					{
					    event_buf[i].flag = 0;
						event_num--;
					}
	            }
	            
	        }
	    }
	}
    return;
}

void init_event_buf(void)
{
    int i = 0;
    for(i=0;i<EVENT_BUFFER_SIZE;i++)
    {
        event_buf[i].flag = 0;
    }
	return;
}
