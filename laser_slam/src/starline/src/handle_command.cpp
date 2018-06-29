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
#include "../include/starline/navigation.h"
#include "../include/starline/handle_command.h"
#include "../include/starline/led.h"

#define PKG_LEN_INDEX  (1)
#define PKG_INDEX_INDEX (3)
#define PKG_TYPE_INDEX  (4)
#define PKG_DATA_INDEX   (6)

#define PKG_BASE_LEN  (26)
#define PKG_CMD_BASE_LEN (9)

inline void restore_long_int_buf(unsigned char *buf,long int *data)
{
    *((char *)(data)) = buf[0];
	*((char *)(data)+1) = buf[1];
	*((char *)(data)+2) = buf[2];
	*((char *)(data)+3) = buf[3];
	*((char *)(data)+4) = buf[4];
	*((char *)(data)+5) = buf[5];
	*((char *)(data)+6) = buf[6];
	*((char *)(data)+7) = buf[7];
}

inline void restore_float_buf(unsigned char *buf,float *data)
{
	*((char *)(data)) = buf[0];
    *((char *)(data)+1) = buf[1];
    *((char *)(data)+2) = buf[2];
    *((char *)(data)+3) = buf[3];
}


inline void set_long_int_buf(unsigned char *buf,long int data)
{
    buf[0] = *((unsigned char *)&data);
    buf[1] = *((unsigned char *)&data+1);
    buf[2] = *((unsigned char *)&data+2);
    buf[3] = *((unsigned char *)&data+3);
	buf[4] = *((unsigned char *)&data+4);
    buf[5] = *((unsigned char *)&data+5);
    buf[6] = *((unsigned char *)&data+6);
	buf[7] = *((unsigned char *)&data+7);
}


inline void set_float_buf(unsigned char *buf,float data)
{
    buf[0] = *((unsigned char *)&data);
    buf[1] = *((unsigned char *)&data+1);
    buf[2] = *((unsigned char *)&data+2);
    buf[3] = *((unsigned char *)&data+3);
}

static void set_robot_state_buf(unsigned char *buf,system_t *sys)
{
    unsigned char ctmp = 0;

    if((NULL == sys) || (NULL == buf))
    {
        ROS_DEBUG("sys or buf NULL!");
        return;
    }
    
    //set system mode state
    if(1 == sys->auto_enable)
    {
        switch(sys->auto_work_mode)
        {
            case AUTO_BASIC_MODE:
                ctmp |= AUTO_BASIC_MASK;
                break;
                
            case AUTO_DANCE_MODE:
                ctmp |= AUTO_DANCE_MASK;
                break;
                
            default:
                break;
        }
    }
    else
    {
        switch(sys->manual_work_mode)
        {
            case MANUAL_BASIC_MODE:
                ctmp |= MANUAL_BASIC_MASK;
                break;
                
            case MANUAL_MAP_MODE:
                ctmp |= MANUAL_MAP_MASK;
                break;
                
            default:
                break;
        }
    }

    //set system err state and warning state

    *buf = ctmp;
    return;
}

int set_system_params(system_t *sys,motion_t *motion,env_t *env,int type,unsigned char *buf)
{
    float data = 0.0;
	int set_flag = 0;
	int i = 0;
		
    if((NULL == sys) || (NULL == motion) || (NULL == env) || (NULL == buf))
    {
        ROS_DEBUG("sys motion env or buf is NULL");
        return -1;
    }

	switch(type)
	{
	    case 0:
			//set camera center value
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			sys->video_to_center.x = data;
			i = write_system_file(sys);
			if(0 == i)
			{
			    set_flag = 1;
			}
			break;
		case 1:
			//set avoid slow limit 
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data >= 0.0)
			{
				sys->sensor.slow_limit= data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 2:
			//set avoid estop limit 
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data >= 0.0)
			{
				sys->sensor.estop_limit = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 3:
			//set camera min limit
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->min_z = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 4:
			//set camera max limit
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->max_z = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 5:
			//set observe range
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->observe_dist = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 6:
			//set tpoint range
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->tolerance_pass = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 7:
			//set goal range
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->tolerance_goal = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 8:
			//set max manual vx
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->max_manual_vx = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 9:
			//set max manual vth
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->max_manual_vth = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 10:
			//set max navigation vx
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->max_vx = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 11:
			//set max navigation vth
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->max_vth = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 12:
			//set max acc_x
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->max_accx = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 13:
			//set max acc_th
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->max_accth = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 14:
			//set dance start point range in line
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->dance.dance_xy_range = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 15:
			//set dance start point range scale
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data >= 1.0)
			{
				sys->dance.dance_xy_scale = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 16:
			//set dance start point range in theta
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
				sys->dance.dance_th_range = data;
				i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 17:
			//set server ip address 
			i = ((buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]));
			i = set_upper_server_ip((unsigned int)i);
			if(0 == i)
			{
			    i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 18:
			//set sonar event limit 
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
			    sys->sensor.sonar_event1_limit = data;
			    i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 19:
			//set manual control overtime 
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data > 0.0)
			{
			    sys->manual_control_over_time = data;
			    i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 20:
			//set camera center value y
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			sys->video_to_center.y = data;
			i = write_system_file(sys);
			if(0 == i)
			{
			    set_flag = 1;
			}
			break;
		case 21:
			//set camera center value th
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			sys->video_to_center.th = data;
			i = write_system_file(sys);
			if(0 == i)
			{
			    set_flag = 1;
			}
			break;
		case 22:
			//enter event type
			i = ((buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]));
			sys->enter_event_type = i;
			i = write_system_file(sys);
			if(0 == i)
			{
			    set_flag = 1;
			}
			break;
		case 23:
			//set enter event limit
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
            if(data >= 0.0)
			{
			    sys->enter_event_limit = data;
			    i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 24:
			//set nav over time 
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data >= 0.0)
			{
			    sys->nav_over_time = data;
			    i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 25:
			//set nav goal nearby range
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data >= 0.0)
			{
			    sys->goal_nearby_range = data;
			    i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
        case 27:
			//enter sonar num
			i = ((buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]));
			sys->enter_sonar_num = i;
			i = write_system_file(sys);
			if(0 == i)
			{
			    set_flag = 1;
			}
			break;
		case 28:
			//set nav goal nearby th
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			if(data >= 0.0)
			{
			    sys->goal_nearby_th = data;
			    i = write_system_file(sys);
				if(0 == i)
				{
				    set_flag = 1;
				}
			}
			break;
		case 200:
			//set reserve_int_1 
			i = ((buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]));
			sys->reserve_int_1 = i;
			i = write_system_file(sys);
			if(0 == i)
			{
			    set_flag = 1;
			}
			break;
		case 201:
			//set reserve_int_2 
			i = ((buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]));
			sys->reserve_int_2 = i;
			i = write_system_file(sys);
			if(0 == i)
			{
			    set_flag = 1;
			}
			break;
		case 202:
			//set reserve_double_3 
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			sys->reserve_double_3 = data;
			i = write_system_file(sys);
			if(0 == i)
			{
			    set_flag = 1;
			}
			break;
		case 203:
			//set reserve_double_4 
			restore_float_buf(buf,&data);
            ROS_DEBUG("type:%d,data:%f",type,data);
			sys->reserve_double_4 = data;
			i = write_system_file(sys);
			if(0 == i)
			{
			    set_flag = 1;
			}
			break;
		default:
			break;
	}

    if(0 == set_flag)
    {
        return -1;
    }
	return 0;
}


//
int send_pkg_back(unsigned short int pkg_type,system_t *sys,motion_t *motion,
          env_t *env,int data,int type,unsigned char *str)
{
    static unsigned char send_num = 1;
    int itmp = 0;
    int start = 0;
	unsigned short int pkg_len = 0;
	unsigned short int check = 0;
    unsigned char send_buf[SOCKET_PKG_LEN] = {0,};
    char tmp_buf[BUF_LEN] = {0,};
    char buf_map[8] = {'X','Y','Z','T','E','M','P','G'};
    
    if((NULL == sys) || (NULL == motion))
    {
        ROS_DEBUG("sys or motion NULL!");
        return -1;
    }
    
    send_buf[0] = 0x55;
	//send_buf[PKG_LEN_INDEX+1] = 0;
    send_buf[PKG_INDEX_INDEX] = send_num;
    send_num++;
    send_buf[PKG_TYPE_INDEX] = pkg_type&0x00ff;   //low byte before high byte
    send_buf[PKG_TYPE_INDEX+1] = (pkg_type&0xff00)>>8;
    switch(pkg_type)
    {
        case PKG_FB_MODE:
            //feedback:current mode
            pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = sys->auto_enable;
            break;
            
        case PKG_FB_REAL_VEL:
            //feedback:real vel
            pkg_len = PKG_BASE_LEN + 4*2;
            itmp = sys->real_vel.vx * 1000.0;
            set_int_buf(&(send_buf[PKG_DATA_INDEX]),itmp);

            itmp = sys->real_vel.vth * 1000.0;    
            set_int_buf(&(send_buf[PKG_DATA_INDEX+4]),itmp);
            break;
            
        case PKG_FB_INIT_MAP:
            //feedback:initial mode status
			pkg_len = PKG_BASE_LEN + 1;
            if(MANUAL_MAP_MODE == sys->manual_work_mode)
            {
                send_buf[PKG_DATA_INDEX] = 1;
            }
            else
            {
                send_buf[PKG_DATA_INDEX] = 0;
            }
            break;
        case PKG_FB_CAL_MARK:
            //feedback:mark flag
            send_buf[PKG_DATA_INDEX] = data;//result
			pkg_len = PKG_BASE_LEN + 1;
            break;
        case PKG_FB_CAL_TPOINT:
            //feedback:tpoint flag
            //send_buf[PKG_LEN_INDEX] = 25;
            pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_CAL_GOAL:
            //feedback:goal flag
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_REPORT_ERROR:
            //report error to do
            //send_buf[PKG_LEN_INDEX] = 28;
			pkg_len = PKG_BASE_LEN + 4;
            itmp = data;
            set_int_buf(&(send_buf[PKG_DATA_INDEX]),itmp);
            break;
        case PKG_FB_CURRENT_GOAL:
            //send_buf[PKG_LEN_INDEX] = 28;
			pkg_len = PKG_BASE_LEN + 4;
            itmp = data;
            set_int_buf(&(send_buf[PKG_DATA_INDEX]),itmp);
            break;
        case PKG_FB_CURRENT_DANCE:
            //start dance cmd feedback
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
		case PKG_FB_REBOOT_SYSTEM:
            //send_buf[PKG_LEN_INDEX] = 24;
			pkg_len = PKG_BASE_LEN;
            break;
        case PKG_FB_SHUTDOWN:
            //send_buf[PKG_LEN_INDEX] = 24;
			pkg_len = PKG_BASE_LEN;
            break;
        case PKG_FB_STOP_DANCE:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_DANCE_STATUS:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_PREPARE_DANCE:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_BACK_DANCE_START_POINT:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_ENTER_DANCE_MODE:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_EXIT_DANCE_MODE:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_PREPARE_NAV:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_NAV_STATUS:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_STOP_NAV:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
            break;
        case PKG_FB_UPDATE_DANCE_START_POINT:
            //send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
            send_buf[PKG_DATA_INDEX] = data;
			break;
		case PKG_FB_SEND_MAP_INFO_BEGIN:
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
            send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			break;
		case PKG_FB_SEND_MAP_INFO:
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
		    break;
		case PKG_FB_SEND_MAP_INFO_END:
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
		    break;
		case PKG_FB_READ_SENSE_DATA:
			//feedback sense data
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
		    switch(type)
		    {
		        case 0:
					itmp = 1;//todo
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),itmp);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 1:
					for(itmp = 0;itmp < LASER_NUM;itmp++)
					{
					    set_float_buf(&(send_buf[PKG_DATA_INDEX+2+itmp*4])
							,(float)sys->sensor.laser_len[itmp]);
					}
					//send_buf[PKG_LEN_INDEX] = 26 + 4*LASER_NUM;
					pkg_len = PKG_BASE_LEN + 2 + 4*LASER_NUM;
					break;
				case 2:
					for(itmp = 0;itmp < SONAR_NUM;itmp++)
					{
					    set_float_buf(&(send_buf[PKG_DATA_INDEX+2+itmp*4])
							,(float)sys->sensor.sonar_len[itmp]);
					}
					//send_buf[PKG_LEN_INDEX] = 26 + 4*SONAR_NUM;
					pkg_len = PKG_BASE_LEN + 2 + 4*SONAR_NUM;
					break;
				case 3:
					send_buf[PKG_DATA_INDEX+2] = sys->sensor.infrared_flag;
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 1;
                    //ROS_DEBUG("infrade flag :%x,%x",send_buf[7],sys->sensor.infrared_flag);
					break;
				case 4:
					itmp = sys->sensor.work_normal;
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),itmp);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 5:
					send_buf[PKG_DATA_INDEX+2] = sys->sensor.estop_io_flag;
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 1;
                    ROS_DEBUG("estop io flag :%x",send_buf[PKG_DATA_INDEX+2]);
					break;
				case 6:
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),
						(float)sys->sensor.estop_fb_limit);
					//send_buf[PKG_LEN_INDEX] = 30;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 7:
					//sensor version
					for(itmp=0;itmp<VERSION_LEN;itmp++)
					{
					    send_buf[PKG_DATA_INDEX+2+itmp] = sys->sensor.version[itmp];
					}
					pkg_len = PKG_BASE_LEN + 2 + VERSION_LEN;
					break;
				default:
					break;
		    }
			break;
		case PKG_FB_READ_BASE_DATA:
			//feedback base data
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			switch(type)
			{
			    case 0:
					itmp = 1;//todo base status
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),itmp);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 1:
					send_buf[PKG_DATA_INDEX+2] = sys->base.estop_sensor_flag;
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 2:
					for(itmp = 0;itmp < BASE_LASER_NUM;itmp++)
					{
					    set_float_buf(&(send_buf[PKG_DATA_INDEX+2+itmp*4]),
							(float)sys->base.laser[itmp]);
					}
					//send_buf[PKG_LEN_INDEX] = 26 + 4*BASE_LASER_NUM;
					pkg_len = PKG_BASE_LEN + 2 + 4*BASE_LASER_NUM;
					break;
				case 3:
					if(sys->base.move_status&0x78)
					{
					    send_buf[PKG_DATA_INDEX+2] = 1;
					}
					else
					{
					    send_buf[PKG_DATA_INDEX+2] = 0;
					}
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 4:
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->base.odom.x);
				    set_float_buf(&(send_buf[PKG_DATA_INDEX+6]),(float)sys->base.odom.y);
				    set_float_buf(&(send_buf[PKG_DATA_INDEX+10]),(float)sys->base.odom.th);
					//send_buf[PKG_LEN_INDEX] = 38;
					pkg_len = PKG_BASE_LEN + 2 + 4*3;
		            break;
				case 5:
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->base.fb_vel.vx);
				    set_float_buf(&(send_buf[PKG_DATA_INDEX+6]),(float)sys->base.fb_vel.vth);
					//send_buf[PKG_LEN_INDEX] = 34;
					pkg_len = PKG_BASE_LEN + 2 + 4*2;
					break;
				case 6:
					//todo base self-test reserved
					itmp = sys->base.work_normal;
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),itmp);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 7:
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->base.fb_cmd_vel.vx);
				    set_float_buf(&(send_buf[PKG_DATA_INDEX+6]),(float)sys->base.fb_cmd_vel.vth);
					//send_buf[PKG_LEN_INDEX] = 34;
					pkg_len = PKG_BASE_LEN + 2 + 4*2;
					break;
				case 8:
					for(itmp=0;itmp<BASE_MOTOR_NUM;itmp++)
					{
					    send_buf[PKG_DATA_INDEX+2+itmp] = sys->base.motor_status[itmp];
					}
					//send_buf[PKG_LEN_INDEX] = 26+BASE_MOTOR_NUM;
					pkg_len = PKG_BASE_LEN + 2 + BASE_MOTOR_NUM;
					break;
				case 9:
					send_buf[PKG_DATA_INDEX+2] = sys->base.move_status;
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 10:
					send_buf[PKG_DATA_INDEX+2] = sys->base.move_rssi;
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 11:
					send_buf[PKG_DATA_INDEX+2] = sys->base.power_v;
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 12:
					//base version
					for(itmp=0;itmp<VERSION_LEN;itmp++)
					{
					    send_buf[PKG_DATA_INDEX+2+itmp] = sys->base.version[itmp];
					}
					pkg_len = PKG_BASE_LEN + 2 + VERSION_LEN;
					break;
				case 13:
					send_buf[PKG_DATA_INDEX+2] = sys->base.move_sensor_state;
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 1;
				default:
					break;
			}
			break;
		case PKG_FB_READ_LED_POWER_DATA:
			//feedback led and power data
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
		    switch(type)
		    {
		        case 0:
					itmp = ((int)sys->led_power.power_status2<<8)|((int)sys->led_power.power_status1);
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),itmp);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 1:
					itmp = 0;
					itmp = ((int)sys->led_power.power_v2<<8)|((int)sys->led_power.power_v1);
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),itmp);
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 2:
					//todo led and power self-test reserved
					itmp =0;
					itmp = sys->led_power.work_normal;
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),itmp);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 3:
					//actual led mode
					send_buf[PKG_DATA_INDEX+2] = sys->led_power.act_mode;
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 4:
					//actual led effect
					send_buf[PKG_DATA_INDEX+2] = sys->led_power.act_effect&0x0ff;
					send_buf[PKG_DATA_INDEX+3] = (sys->led_power.act_effect&0xff00)>>8;
					//send_buf[PKG_LEN_INDEX] = 28;
					pkg_len = PKG_BASE_LEN + 2 + 2;
					break;
				case 5:
					//power switch status
					itmp = 0;
					itmp = (((int)sys->led_power.power_switch_status4)<<24)|(((int)sys->led_power.power_switch_status1)<<16)
						       |(((int)sys->led_power.power_switch_status1)<<8)|((int)sys->led_power.power_switch_status1);
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),itmp);
					//send_buf[PKG_LEN_INDEX] = 27;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 6:
					//power current 
					for(itmp=0;itmp<POWER_CURRENT_NUM*2;itmp++)
					{
					    send_buf[PKG_DATA_INDEX+2+itmp] = sys->led_power.power_i[itmp];
					}
					//send_buf[PKG_LEN_INDEX] = 26 + POWER_CURRENT_NUM;
					pkg_len = PKG_BASE_LEN + 2 + POWER_CURRENT_NUM*2;
					break;
				case 7:
					//get power temp current info
					pkg_len = PKG_BASE_LEN + 2;
					set_get_power_flag();
				    break;
				case 8:
					//power version
					for(itmp=0;itmp<VERSION_LEN;itmp++)
					{
					    send_buf[PKG_DATA_INDEX+2+itmp] = sys->led_power.version[itmp];
					}
					pkg_len = PKG_BASE_LEN + 2 + VERSION_LEN;
					break;
				case 9:
					//set led red long
					pkg_len = PKG_BASE_LEN + 2;
					sys->led_power.mmi_test_flag = 1;
					set_led_power_effect(LED_POWER_FREEDOM,LED_RED_LONG);
				    break;
				case 10:
					//set led green long
					pkg_len = PKG_BASE_LEN + 2;
					sys->led_power.mmi_test_flag = 1;
					set_led_power_effect(LED_POWER_FREEDOM,LED_GREEN_LONG);
				    break;
				case 11:
					//set led blue long
					pkg_len = PKG_BASE_LEN + 2;
					sys->led_power.mmi_test_flag = 1;
					set_led_power_effect(LED_POWER_FREEDOM,LED_BLUE_LONG);
				    break;
				case 12:
					//shut down mmi led test
					pkg_len = PKG_BASE_LEN + 2;
					sys->led_power.mmi_test_flag = 0;
					set_led_power_effect(LED_POWER_FREEDOM,LED_NORMAL);
				    break;
				default:
					break;
		    }
			break;
		case PKG_FB_READ_CAMERA_DATA:
			//read camera data
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			switch(type)
			{
			    case 0:
					//camera state.
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->camera.state);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 1:
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->camera.id);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+6]),(float)sys->camera.point.x);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+10]),(float)sys->camera.point.y);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+14]),(float)sys->camera.point.z);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+18]),(float)sys->camera.point.th);
					//send_buf[PKG_LEN_INDEX] = 46;//26+4*5
					pkg_len = PKG_BASE_LEN + 2 + 4*5;
					break;
				default:
					break;
			}
			break;
		case PKG_FB_READ_CONTROLLER_DATA:
			//read controller data
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			switch(type)
			{
			    case 0:
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->sys_status);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 1:
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->err_num);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 2:
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->warn_num);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 3:
					//obstacle scale
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->obstacle_scale);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 4:
					//nav finish status
					send_buf[PKG_DATA_INDEX+2] = motion->path.path_finish;
					//send_buf[PKG_LEN_INDEX] = 26+1;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 5:
					//nav goal id
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),motion->path.goal_id);
					//send_buf[PKG_LEN_INDEX] = 26 + 4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 6:
					//dance finish status
					send_buf[PKG_DATA_INDEX+2] = sys->dance.dance_state;
					//send_buf[PKG_LEN_INDEX] = 26+1;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 7:
					//dance id
					set_long_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->dance.dance_id);
					//send_buf[PKG_LEN_INDEX] = 26 + 8;
					pkg_len = PKG_BASE_LEN + 2 + 8;
					break;
				case 8:
					// build cord flag
					send_buf[PKG_DATA_INDEX+2] = sys->build_cord_flag;
					//send_buf[PKG_LEN_INDEX] = 26+1;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 9:
					//current position
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)motion->current.x);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+6]),(float)motion->current.y);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+10]),(float)motion->current.z);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+14]),(float)motion->current.th);
					//send_buf[PKG_LEN_INDEX] = 42;
					pkg_len = PKG_BASE_LEN + 2 + 4*4;
					break;
				case 10:
					//dance start point
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->dance.dance_start_point.x);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+6]),(float)sys->dance.dance_start_point.y);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+10]),(float)sys->dance.dance_start_point.z);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+14]),(float)sys->dance.dance_start_point.th);
					//send_buf[PKG_LEN_INDEX] = 42;
					pkg_len = PKG_BASE_LEN + 2 + 4*4;
					break;
				case 11:
					// build cord flag
					send_buf[PKG_DATA_INDEX+2] = sys->dance.dance_start_point_set;
					//send_buf[PKG_LEN_INDEX] = 26+1;
					pkg_len = PKG_BASE_LEN + 2 + 1;
					break;
				case 12:
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->real_vel.vx);
					set_float_buf(&(send_buf[PKG_DATA_INDEX+6]),(float)sys->real_vel.vth);
					//send_buf[PKG_LEN_INDEX] = 26+8;
					pkg_len = PKG_BASE_LEN + 2 + 4*2;
					break;
				case 13:
					//video center x
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->video_to_center.x);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 14:
					//slow dist
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->sensor.slow_limit);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 15:
					//estop dist
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->sensor.estop_limit);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 16:
					//min z
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->min_z);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 17:
					//max z
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->max_z);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 18:
					//observe dist
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->observe_dist);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 19:
					//tpoint tolerance
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->tolerance_pass);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 20:
					//goal tolerance
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->tolerance_goal);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 21:
					//max manual vx
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->max_manual_vx);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 22:
					//max manual vth
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->max_manual_vth);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 23:
					//max auto vx
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->max_vx);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 24:
					//max auto vth
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->max_vth);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 25:
					//max acc_x
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->max_accx);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 26:
					//max acc_th
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->max_accth);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 27:
					//dance start point xy range
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->dance.dance_xy_range);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 28:
					//dance start point scale
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->dance.dance_xy_scale);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 29:
					//dance start point th range
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->dance.dance_th_range);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 30:
					//upper controller ip
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),(int)get_upper_server_ip());
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 31:
					//internal develop version num
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),DEVELOP_VERSION_CODE);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 32:
					//official version num
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),OFFICIAL_VERSION_CODE);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 33:
					//sonar event limit
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->sensor.sonar_event1_limit);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 34:
					//manual control overtime
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)(sys->manual_control_over_time));
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 35:
					//video center y
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->video_to_center.y);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 36:
					//video center th
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->video_to_center.th);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 37:
					//enter event type
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->enter_event_type);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 38:
					//enter event limit
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->enter_event_limit);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 39:
					//nav over time 
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->nav_over_time);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 40:
					//nav goal nearby range
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->goal_nearby_range);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 41:
					//upgrade overtime range
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),UPGRADE_OVER_TIME);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
                    break;
				case 42:
					//enter sonar num
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->enter_sonar_num);
					//send_buf[1] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
                case 43:
					//nav goal nearby th
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->goal_nearby_th);
                    //send_buf[1] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 44:
					//product id 
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),PRODUCT_ID);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
                    break;
				case 45:
					//system type id 
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),SYSTEM_TYPE_ID);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
                    break;
				case 200:
					//reserve_int_1
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->reserve_int_1);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 201:
					//reserve_int_1
					set_int_buf(&(send_buf[PKG_DATA_INDEX+2]),sys->reserve_int_2);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 202:
					//reserve_double_3
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->reserve_double_3);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				case 203:
					//reserve_double_4
					set_float_buf(&(send_buf[PKG_DATA_INDEX+2]),(float)sys->reserve_double_4);
					//send_buf[PKG_LEN_INDEX] = 26+4;
					pkg_len = PKG_BASE_LEN + 2 + 4;
					break;
				default:
					break;
			}
			break;
		case PKG_FB_SET_CONTROLLER_PARAMS:
			//set controller params
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
			break;
		case PKG_FB_SEND_DANCE_FILE_BEGIN:
			//feedback dance file begin
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
			break;
		case PKG_FB_SEND_DANCE_FILE:
			//feedback dance file send
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
			break;
		case PKG_FB_SEND_DANCE_FILE_END:
			//feedback dance file end
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
			break;
		case PKG_FB_SEND_UPGRADE_FILE_BEGIN:
			//upgrade begin
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
			break;
		case PKG_FB_SEND_UPGRADE_FILE:
			//upgrade 
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
			break;
		case PKG_FB_SEND_UPGRADE_FILE_END:
			//upgrade end
			send_buf[PKG_DATA_INDEX] = data;
			send_buf[PKG_DATA_INDEX+1] = type;
			//send_buf[PKG_LEN_INDEX] = 26;
			pkg_len = PKG_BASE_LEN + 2;
			break;
		case PKG_FB_READ_DEVELOP_VERSION:
			//version 
			//send_buf[PKG_LEN_INDEX] = 28;
			pkg_len = PKG_BASE_LEN + 4;
            set_int_buf(&(send_buf[PKG_DATA_INDEX]),DEVELOP_VERSION_CODE);
			break;
		case PKG_FB_CHECK_SYSTEM:
			//check system
			//send_buf[PKG_LEN_INDEX] = 24+data;
			pkg_len = PKG_BASE_LEN + data;
			for(itmp=0;itmp<data;itmp++)
			{
			    send_buf[PKG_DATA_INDEX+itmp] = str[itmp];
			}
			break;
		case PKG_FB_READ_OFFICIAL_VERSION:
			//official version
			//send_buf[PKG_LEN_INDEX] = 28;
			pkg_len = PKG_BASE_LEN + 4;
            set_int_buf(&(send_buf[PKG_DATA_INDEX]),OFFICIAL_VERSION_CODE);
			break;
		case PKG_SEND_HEART_BEAT:
			//send_buf[PKG_LEN_INDEX] = 24;
			pkg_len = PKG_BASE_LEN;
			break;
		case PKG_FB_REBOOT_ROBOT:
			//send_buf[PKG_LEN_INDEX] = 24;
			pkg_len = PKG_BASE_LEN;
			break;
		case PKG_FB_RELOAD_MAP_FILES:
			ROS_DEBUG("reload map files result:%d",data);
			send_buf[PKG_DATA_INDEX] = data;
			//send_buf[PKG_LEN_INDEX] = 25;
			pkg_len = PKG_BASE_LEN + 1;
			break;
        case PKG_FB_REQUEST_UPGRADE:
            pkg_len = PKG_BASE_LEN + 4;
            set_int_buf(&(send_buf[PKG_DATA_INDEX]),data);
            break;
        case PKG_FB_START_UPGRADE:
            pkg_len = PKG_BASE_LEN + 4;
            set_int_buf(&(send_buf[PKG_DATA_INDEX]),data);
		case PKG_REPORT_NAV_FINISHED:
			//send_buf[PKG_LEN_INDEX] = 28;
			pkg_len = PKG_BASE_LEN + 4;
            //goal finished 
            set_int_buf(&(send_buf[PKG_DATA_INDEX]),motion->path.goal_id);
			break;
		case PKG_REPORT_DANCE_FINISHED:
			//send_buf[PKG_LEN_INDEX] = 32;
			pkg_len = PKG_BASE_LEN + 8;
            //dance finished 
            set_long_int_buf(&(send_buf[PKG_DATA_INDEX]),sys->dance.dance_id);
			break;
		case PKG_REPORT_SHUTDOWN:
			//send_buf[PKG_LEN_INDEX] = 24;
			pkg_len = PKG_BASE_LEN;
			//power close,shut down 
			break;
		case PKG_REPORT_AI_WORDS:
			//send_buf[PKG_LEN_INDEX] = 24+data;
			pkg_len = PKG_BASE_LEN + data;
			memcpy(&(send_buf[PKG_DATA_INDEX]),str,data);
			ROS_DEBUG("send ai words");
			break;
        default:
            break;
    }

    send_buf[PKG_LEN_INDEX] = pkg_len&0x0ff;
	send_buf[PKG_LEN_INDEX+1] = (pkg_len&0xff00)>>8;
    //start = send_buf[PKG_LEN_INDEX] - 19;
    itmp = sizeof(float)*4+1+2+1;
    start = pkg_len - itmp;
    set_float_buf(&(send_buf[start]),(float)motion->current.x);
    set_float_buf(&(send_buf[start+4]),(float)motion->current.y);
    set_float_buf(&(send_buf[start+8]),(float)motion->current.z);
    set_float_buf(&(send_buf[start+12]),(float)motion->current.th);

    set_robot_state_buf(&(send_buf[start+16]),sys);
    
    //send_buf[start+17] = 0;
    check = 0;
    for(itmp = 0;itmp < pkg_len - 3;itmp++)
    {
        //ROS_DEBUG("send_buf[%d]:%x",itmp,send_buf[itmp]);
        check += send_buf[itmp];
    }
	send_buf[pkg_len - 3] = check&0x0ff;
	send_buf[pkg_len - 2] = (check&0xff00)>>8;
	
    send_buf[pkg_len - 1] = 0xaa;
    for(int i=0;i<pkg_len;i++)
{
    //ROS_DEBUG("send_pkg_back:%02x",send_buf[i]);
}	
    itmp = send_status_back(send_buf, (int)pkg_len);
    
    return itmp;
}


static int handle_cmd(unsigned char *buf,system_t*sys,motion_t *motion,env_t *env)
{
    int itmp = 0;
    int data = 0;
    int fb_data = 0;
    int type = 0;
    int i = 0;
    int j = 0;
	long int index = 0;
    unsigned short int pkg_type = 0;
	unsigned short int pkg_len = 0;
    unsigned char cdata[SOCKET_PKG_BUF_SIZE] = {0,};
	ask_status_t ask;
	ans_status_t ans;
    
    if((NULL == buf) || (NULL == sys) || (NULL == motion) || (NULL == env))
    {
        ROS_DEBUG("handle_cmd:buf,sys,motion,env is NULL");
        return -1;
    }
    pkg_len = (buf[PKG_LEN_INDEX+1]<<8)|(buf[PKG_LEN_INDEX]);
    pkg_type = (buf[PKG_TYPE_INDEX+1]<<8)|(buf[PKG_TYPE_INDEX]);

    if(1 == motion->path.info_flag)
    {
        motion->path.info_flag = 0;
    }
    if(1 == sys->dance.dance_info_flag)
    {
        sys->dance.dance_info_flag = 0;
    }

    switch(pkg_type)
    {
        case PKG_SET_MODE:
            //cmd:set auto mode or manual mode
            data = buf[PKG_DATA_INDEX];
            if(0 == data)
            {
                //if((1 == sys->auto_enable) && (AUTO_BASIC_MODE == sys->auto_work_mode))
                if(1 == sys->auto_enable)
                {
                    sys->auto_enable = 0;
                    sys->auto_work_mode = AUTO_BASIC_MODE;
					fb_data = 1;
                }
            }
            else
            {
                if((0 == sys->auto_enable) && (MANUAL_BASIC_MODE == sys->manual_work_mode))
                {
                    sys->auto_enable = 1;
					fb_data = 1;
                }
            }
            ROS_DEBUG("handle_cmd:set auto enable:%d",data);
            break;
        case PKG_SET_MANUAL_VEL:
            //cmd:set vel
            if(0 == sys->auto_enable)
            {
                sys->manual_overtime_flag = 0;
                data = (((int)buf[PKG_DATA_INDEX+3])<<24)|
					(((int)buf[PKG_DATA_INDEX+2])<<16)|
					(((int)buf[PKG_DATA_INDEX+1])<<8)|
					((int)buf[PKG_DATA_INDEX]); 
                sys->tele_vel.vx = ((double)data)/1000.0;     //manual vel of x axes
                if(fabs(sys->tele_vel.vx) > sys->max_manual_vx)
                {
                    sys->tele_vel.vx = sys->max_manual_vx * sys->tele_vel.vx 
						/fabs(sys->tele_vel.vx);
                }
                
                data = (((int)buf[PKG_DATA_INDEX+7])<<24)|
					(((int)buf[PKG_DATA_INDEX+6])<<16)|
					(((int)buf[PKG_DATA_INDEX+5])<<8)|
					((int)buf[PKG_DATA_INDEX+4]); 
                sys->tele_vel.vth = ((double)data)/1000.0;    //manual vel of th rotation axes
                if(fabs(sys->tele_vel.vth) > sys->max_manual_vth)
                {
                    sys->tele_vel.vth = sys->max_manual_vth * sys->tele_vel.vth
						/fabs(sys->tele_vel.vth);
                }
                fb_data = 1;
            }
            ROS_DEBUG("handle_cmd:set vx:%f,vth:%f",sys->tele_vel.vx,sys->tele_vel.vth);
            break;
        case PKG_ENABLE_INIT_MAP:
            //cmd:set initial map mode
            data = buf[PKG_DATA_INDEX];
            if(1 == data)
            {
                if((0 == sys->auto_enable) && (MANUAL_BASIC_MODE == sys->manual_work_mode))
                {
                    sys->manual_work_mode = MANUAL_MAP_MODE;
					fb_data = 1;
                }
            }
            else
            {
                if((0 == sys->auto_enable) && (MANUAL_MAP_MODE == sys->manual_work_mode))
                {
                    sys->manual_work_mode = MANUAL_BASIC_MODE;
					fb_data = 1;
                }
            }
            ROS_DEBUG("handle_cmd:initial map:%d",data);
            break;
        case PKG_CAL_MARK:
            //cmd:get a mark point
            ROS_DEBUG("handle_cmd:save mark to g_env.env_mark");
            if(MANUAL_MAP_MODE == sys->manual_work_mode)
            {
                //save mark to g_env.env_mark
                sys->set_mark_flag = 1;
                sys->mark_count = 1;
                return 0;
            }
            break;
        case PKG_CAL_TPOINT:
            //cmd:get a tpoint point 
            ROS_DEBUG("handle_cmd:save tpoint,tpoint num:%d",sys->tpoint_num);
            if(MANUAL_MAP_MODE == sys->manual_work_mode)
            {
                //i = find_unused_tpoint(sys,env);
                //j = check_same_tpoint(sys,env,&(motion->current));
                if((i >= 0) && (i < sys->tpoint_num) && (0 == j))
                {
                    env->env_tpoint[i].point.x = motion->current.x;
                    env->env_tpoint[i].point.y = motion->current.y;
                    env->env_tpoint[i].point.z = motion->current.z;
                    env->env_tpoint[i].point.th = motion->current.th;
                    env->env_tpoint[i].set_flag = 1;
                    env->env_tpoint[i].id = sys->tpoint_count;
                    sys->tpoint_count++;
                    fb_data = 1;
                    type = i;
                }
                else
                {
                    ROS_DEBUG("save tpoint to g_env.env_tpoint failed!\n");
                }
            }
			ROS_DEBUG("handle_cmd:save tpoint fb_data:%d,i:%d\n",fb_data,i);
            break;
        case PKG_CAL_GOAL:
            //0x06,cmd:get a goal point
            ROS_DEBUG("handle_cmd:save goal,goal num:%d",sys->goal_num);
            if(MANUAL_MAP_MODE == sys->manual_work_mode)
            {
                //i = find_unused_goal(sys,env);
                //j = check_same_goal(sys,env,&(motion->current));
                if((i >= 0) && (i < sys->goal_num) && (0 == j))
                {
                    env->env_goal[i].point.x = motion->current.x;
                    env->env_goal[i].point.y = motion->current.y;
                    env->env_goal[i].point.z = motion->current.z;
                    env->env_goal[i].point.th = motion->current.th;
                    env->env_goal[i].set_flag = 1;
                    env->env_goal[i].id = sys->goal_count;
                    sys->goal_count++;
                    fb_data = 1;
                    type = i;
                }
                else
                {
                    ROS_DEBUG("save goal to g_env.env_goal failed!\n");
                }
            }
			ROS_DEBUG("handle_cmd:save goal fb_data:%d,i:%d\n",fb_data,i);
            break;
        case PKG_SET_NAV_GOAL:
			ROS_DEBUG("handle_cmd:set navigation goal");
            data = (((int)buf[PKG_DATA_INDEX+3])<<24)|
				(((int)buf[PKG_DATA_INDEX+2])<<16)|
				(((int)buf[PKG_DATA_INDEX+1])<<8)|((int)buf[PKG_DATA_INDEX]);
            fb_data = -1;
            if((data >= 0) && (1 == sys->auto_enable) 
				&& (AUTO_BASIC_MODE == sys->auto_work_mode))
            {
                //itmp = find_goal_index_by_id(env->env_goal,data,sys->goal_num);
                if(sys->goal_num == itmp)
                {
                    ROS_DEBUG("set nav goal failed,can not find goal:%x",data);
                    break;
                }
                ROS_DEBUG("setup goal[%d]:%d for goal!\n",itmp,data);
                //itmp = cal_path(motion,itmp,env,sys);
                if(0 == itmp)
                {
                    fb_data = data;
					motion->path.goal_id = data;
                    set_led_power_effect(LED_POWER_FREEDOM,LED_NAV);
                }
            }
            break;
        case PKG_START_DANCE:
			ROS_DEBUG("handle_cmd:start dance");
            if((1 == sys->auto_enable)&&(AUTO_DANCE_MODE == sys->auto_work_mode))
            {
				//get dance index and find dance file,then read dance file and load to sys->dance struct.
			    if(DANCE_FINISHED == sys->dance.dance_state)
			    {
					restore_long_int_buf(&(buf[PKG_DATA_INDEX]),&index);
					//itmp = load_dance_file(sys,index);
					if(0 == itmp)
					{
					    if(0 == sys->dance.dance_start_point_set)
	                    {
	                        //set_dance_start_point(sys,motion);
	                    }
	                    sys->dance.dance_state = DANCING;
						sys->dance.dance_seq.time = 0.0;
						sys->dance.dance_seq.start_time = 0.0;
						sys->dance.dance_rotate_flag = 0;
						sys->dance.led_seq.set_led_effect_flag = 0;
						sys->dance.dance_seq.dance_move_index = 0;
					    fb_data = 1;
						//for dance everywhere and when,even if pushed or estop key
						//sys->event_stop_flag = 0;
					}
			    }
				else
				{
				    ROS_DEBUG("dance unfinished!");
				}
            }
            else
            {
                ROS_DEBUG("system is not  in dance mode");
            }
			ROS_DEBUG("handle set dance:%d",fb_data);
            break;
		case PKG_REBOOT_SYSTEM:
			ROS_DEBUG("reboot system");
			send_pkg_back(PKG_FB_REBOOT_SYSTEM,sys,motion,env,0,0,NULL);
			sleep(2);
			itmp = system("sudo shutdown -r now");
			ROS_DEBUG("reboot navigation controller,result:%d",itmp);
			break;
        case PKG_SHUTDOWN_SYSTEM:
            ROS_DEBUG("shutdown the system");
            send_pkg_back(PKG_FB_SHUTDOWN,sys,motion,env,0,0,NULL);
			//set_led_power_effect(LED_POWER_FREEDOM,LED_DEFAULT);
            //sleep(2);
            //itmp = system("sudo shutdown -h now");
			//ROS_DEBUG("shut down navigation controller,result:%d",itmp);
            break;
        case PKG_STOP_DANCE:
            ROS_DEBUG("stop dance mode");
            if((1 == sys->auto_enable) && (AUTO_DANCE_MODE == sys->auto_work_mode))
            {
                sys->dance.dance_state = DANCE_FINISHED;
                sys->dance.dance_rotate_flag = 0;
                fb_data = 1;
            }
            break;
        case PKG_REQUEST_DANCE_STATUS:
            ROS_DEBUG("request dance state");
            if((1 == sys->auto_enable) && (AUTO_DANCE_MODE == sys->auto_work_mode)
                && (DANCE_ERROR != sys->dance.dance_state))
            {
                fb_data = 0;
            }
            else
            {
                fb_data = 1;
            }
            break;
        case PKG_ASK_PREPARE_DANCE:
            ROS_DEBUG("handle_cmd: ask dance prepare ok?");
            //fb_data = check_dance_status(motion,sys);
            break;
        case PKG_BACK_DANCE_START_POINT:
            ROS_DEBUG("handle_cmd:back start point");
            if((1 == sys->auto_enable) && (AUTO_DANCE_MODE == sys->auto_work_mode))
            {
                if((0 != sys->dance.dance_start_point_set) 
						&& (DANCE_FINISHED == sys->dance.dance_state))
                {
                    //
                    ROS_DEBUG("dance back to start point now!");
                    sys->dance.dance_state = DANCE_BACK_START;
                    fb_data = 1;
                }
            }
            break;
        case PKG_ENTER_DANCE_MODE:
			ROS_DEBUG("handle_cmd:enter dance mode");
            if((AUTO_BASIC_MODE == sys->auto_work_mode)&&(1 == sys->auto_enable))
            {
                sys->auto_work_mode = AUTO_DANCE_MODE;
                fb_data = 1;
            }
            else if((AUTO_DANCE_MODE == sys->auto_work_mode)&&(1 == sys->auto_enable))
            {
                fb_data = 1;
            }
            break;
        case PKG_EXIT_DANCE_MODE:
			ROS_DEBUG("exit dance mode!");
            if((AUTO_DANCE_MODE == sys->auto_work_mode)&&(1 == sys->auto_enable))
            {
                sys->auto_work_mode = AUTO_BASIC_MODE;
                sys->dance.dance_state = DANCE_FINISHED;
                sys->dance.dance_rotate_flag = 0;
                fb_data = 1;
            }
            break;
        case PKG_ASK_PREPARE_NAV:
            ROS_DEBUG("handle_cmd: ask nav prepare ok?");
			fb_data = 0;
            if((1 != sys->auto_enable) || (AUTO_BASIC_MODE != sys->auto_work_mode))
            {
                ROS_DEBUG("system mode is not right");
                fb_data = 1;
            }
            else
            {
                if(0 == motion->path.path_finish)
                {
                    ROS_DEBUG("last nav unfinished");
                    fb_data = 2;
                }
                else if(0 == sys->build_cord_flag)
                {
                    ROS_DEBUG("system not find valid mark");
                    fb_data = 3;
                }
				else
				{
					if(1 == sys->camera.lose_mark_flag)
	                {
	                    ROS_DEBUG("system lost mark");
	                    fb_data = 100;
	                }
				}
            }
            break;
        case PKG_REQUEST_NAV_STATUS:
            ROS_DEBUG("handle_cmd: request nav state");
            if((1 != sys->auto_enable) || (AUTO_BASIC_MODE != sys->auto_work_mode))
            {
                ROS_DEBUG("system mode is not right,nav failed");
                fb_data = 2;
            }
            else if(sys->obstacle_scale < OBSTACLE_SLOW_SCALE)
            {
                ROS_DEBUG("nav ok,but find obstacle");
                fb_data = 1;
            }    
            break;
        case PKG_STOP_NAV:
            ROS_DEBUG("handle_cmd: stop nav");
            if((AUTO_BASIC_MODE == sys->auto_work_mode) && (1 == sys->auto_enable))
            {
                motion->path.path_finish = 1;
                fb_data = 1;
            }
            break;
        case PKG_UPDATE_DANCE_START_POINT:
            ROS_DEBUG("handle_cmd: setup new dance start point");
            if(1 == sys->build_cord_flag)
            {
                //set_dance_start_point(sys,motion);
                i = write_system_file(sys);
				if(0 == i)
				{
					//i = read_system_file(sys);
				    if(0 == i)
				    {
				        fb_data = 1;
				    }
					else
					{
					    ROS_DEBUG("re-read system file failed!");
                        fb_data = 2;
					}
				}
				else
				{
				    ROS_DEBUG("write to system file failed!");
				    sys->err_num = WRITE_SYS_FILE_ERR;
                    fb_data = 3;
				}
            }
            else
            {
                fb_data = 4;
            }
            break;
        case PKG_SEND_MAP_INFO_BEGIN:
            ROS_DEBUG("handle_cmd:begin send map file");
            type = buf[PKG_DATA_INDEX];
            data = (((int)buf[PKG_DATA_INDEX+4])<<24)|
				(((int)buf[PKG_DATA_INDEX+3])<<16)|
				(((int)buf[PKG_DATA_INDEX+2])<<8)|((int)buf[PKG_DATA_INDEX+1]);
           // i = handle_map_files(sys,env,(FILE_TYPE)type,data);
			if(0 == i)
			{
			    fb_data = 1;
			}
			break;
		case PKG_SEND_MAP_INFO:
			ROS_DEBUG("handle_cmd:send map file");
			type = buf[PKG_DATA_INDEX];
			itmp = pkg_len - PKG_CMD_BASE_LEN - 1;
			//i = copy_map_files_mem(env,(char *)&(buf[PKG_DATA_INDEX+1]),itmp,(FILE_TYPE)type);
			if(0 == i)
			{
			    fb_data = 1;
			}
			break;
		case PKG_SEND_MAP_INFO_END:
			ROS_DEBUG("handle_cmd: send map file end");
			type = buf[PKG_DATA_INDEX];
			//i = check_map_file(env,(char)buf[PKG_DATA_INDEX+1],(FILE_TYPE)type);
			ROS_DEBUG("check map file:%d",i);
			if(0 == i)
			{
			    //i = write_to_map_files(env,(FILE_TYPE)type);
				if(0 == i)
				{
				    fb_data = 1;
				}
				else
				{
				    ROS_DEBUG("write to map files failed!");
				    sys->err_num = WRITE_MAP_FILE_ERR;
				}
			}
			else
			{
			    ROS_DEBUG("check map file failed!");
			}
			break;
		case PKG_READ_SENSE_DATA:
			ROS_DEBUG("handle_cmd: read sensor data");
			fb_data = 1;
			type = buf[PKG_DATA_INDEX];
			break;
		case PKG_READ_BASE_DATA:
			ROS_DEBUG("handle_cmd: read base data");
			fb_data = 1;
			type = buf[PKG_DATA_INDEX];
			break;
		case PKG_READ_LED_POWER_DATA:
			ROS_DEBUG("handle_cmd: read led and power data");
			fb_data = 1;
			type = buf[PKG_DATA_INDEX];
			break;
		case PKG_READ_CAMERA_DATA:
			ROS_DEBUG("handle_cmd: read camera data");
			fb_data = 1;
			type = buf[PKG_DATA_INDEX];
			break;
		case PKG_READ_CONTROLLER_DATA:
			ROS_DEBUG("handle_cmd: read controller data,type:%d",buf[PKG_DATA_INDEX]);
			fb_data = 1;
			type = buf[PKG_DATA_INDEX];
			break;
		case PKG_SET_CONTROLLER_DATA:
			ROS_DEBUG("handle_cmd: set controller params,type:%d",buf[PKG_DATA_INDEX]);
			type = buf[PKG_DATA_INDEX];
			i = set_system_params(sys,motion,env,type,&(buf[PKG_DATA_INDEX+1]));
			if(0 == i)
			{
			    fb_data = 1;
			}
			break;
		case PKG_SEND_DANCE_FILE_BEGIN:
			ROS_DEBUG("handle_cmd: send dance file begin");
			type = buf[PKG_DATA_INDEX];
			//i = handle_dance_file_init(sys, buf);
			if(0 == i)
			{
			    fb_data = 1;
			}
			break;
		case PKG_SEND_DANCE_FILE:
			ROS_DEBUG("handle_cmd: send dance file");
			type = buf[PKG_DATA_INDEX];
			//i = handle_dance_file_data(sys, buf);
			if(0 == i)
			{
			    fb_data = 1;
			}
			break;
		case PKG_SEND_DANCE_FILE_END:
			ROS_DEBUG("handle_cmd: send dance file end");
			type = buf[PKG_DATA_INDEX];
			//i = handle_dance_file_done(sys, buf);
			if(0 == i)
			{
			    fb_data = 1;
			}
		    break;
		case PKG_SEND_UPGRADE_FILE_BEGIN:
			ROS_DEBUG("handle_cmd:send upgrade file begin");
			type = buf[PKG_DATA_INDEX];
			i = handle_upgrade_file_begin(sys,buf);
			if(0 == i)
			{
			    fb_data = 1;
			}
			break;
		case PKG_SEND_UPGRADE_FILE:
			ROS_DEBUG("handle_cmd: send upgrade file");
			type = buf[PKG_DATA_INDEX];
			i = handle_upgrade_file_data(sys, buf);
			if(0 == i)
			{
			    fb_data = 1;
			}
			break;
		case PKG_SEND_UPGRADE_FILE_END:
			ROS_DEBUG("handle_cmd: send upgrade file end");
			type = buf[PKG_DATA_INDEX];
			i = handle_upgrade_file_done(sys, buf);
			if(0 == i)
			{
				if(SYSTEM_FILE == sys->upgrade.type)
				{
				    i = upgrade_replace_nav_file();
					if(0 == i)
					{
					    fb_data = 1;
					}
				}
			}
			else
			{
			    ROS_DEBUG("write upgrade file failed");
			}
			break;
		case PKG_READ_DEVELOP_VERSION:
			ROS_DEBUG("handle_cmd: read develop version");
			fb_data = 1;
			break;
		case PKG_CHECK_SYSTEM:
			ROS_DEBUG("handle_cmd: check system");
			ask.module_group = (module_e)buf[PKG_DATA_INDEX];
			ask.module = (module_e)buf[PKG_DATA_INDEX+1];
			ask.function = buf[PKG_DATA_INDEX+2];
			i = handle_report_status(ask,&ans,sys,motion,env);
			if(0 != i)
			{
			    ans.level = LEVEL_ERROR;
				ans.module = MODULE_NAV;
				ans.function = 23;
				ans.len = 4;
				set_int_buf(ans.data,4);
			}
			cdata[0] = (unsigned char)ans.level;
			cdata[1] = (unsigned char)ans.module;
			cdata[2] = ans.function;
			cdata[3] = ans.len;
			for(i=0;i<ans.len;i++)
			{
			    cdata[4+i] = ans.data[i];
			}
			fb_data = ans.len + 4;
            ROS_DEBUG("check system:module:%x,function:%x",cdata[1],cdata[2]);
			break;
		case PKG_READ_OFFICIAL_VERSION:
			ROS_DEBUG("handle_cmd: read official version");
			fb_data = 1;
			break;
		case PKG_HEART_BEAT_FB:
			ROS_DEBUG("handle_cmd:receive heart beat back pkg");
			set_upper_beat_flag(1);
			return 0;
			break;
		case PKG_REBOOT_ROBOT:
			ROS_DEBUG("handle_cmd:reboot robot");//
            i = set_led_power_function(0,1);
            if(0 == i)
            {
                fb_data = 1;
            }
			break;
		case PKG_RELOAD_MAP_FILES:
			ROS_DEBUG("handle_cmd:reload map files");
			//i = reload_map_files(sys,env);
			if(0 == i)
			{
			    fb_data = 1;
			}
			break;
		case PKG_REQUEST_UPGRADE:
			ROS_DEBUG("handle_cmd:request upgrade");
			fb_data = 0;
			if(pkg_len < SOCKET_PKG_BUF_SIZE)
			{
				memcpy(cdata,&(buf[PKG_DATA_INDEX]),pkg_len-PKG_CMD_BASE_LEN);
				cdata[pkg_len-PKG_CMD_BASE_LEN] = 0;
				i = check_upgrade_system(sys,env,cdata,&type);
                ROS_DEBUG("check_upgrade_system result:%d",i);
				if(0 == i)
				{
				    fb_data |= 0x01;
				}
				if(1 == type)
				{
				    fb_data |= 0x02;
				}
			}
			break;
		case PKG_START_UPGRADE:
			ROS_DEBUG("handle_cmd:start upgrade");
			sys->system_upgrade_flag = 1;
            fb_data = 0;
			break;
	    default:
			ROS_DEBUG("handle_cmd:default:%x",pkg_type);
			return -1;
            break;
    }
    ROS_DEBUG("pkg_type:%x,fbdata:%d",pkg_type,fb_data);
    send_pkg_back(pkg_type|PKG_TRANSFORM,sys,motion,env,fb_data,type,cdata);

    return 0;
}

void handle_upper_com_cmd(system_t *sys,motion_t *motion,env_t *env)
{
    upper_com_sys_t *upper_sys = NULL;
    unsigned char *buf = NULL;
    int i = 0;
    unsigned short int j = 0;
    int k = 0;
    int num = 0;
    int buf_start = 0;
	unsigned short int check_sum = 0;
	unsigned short int check = 0;

    if((NULL == sys) || (NULL == motion) ||(NULL == env))
    {
        ROS_DEBUG("sys or motion or env NULL!");
        return;
    }
    
    //read upper_com  info to decide handle
    upper_sys = get_upper_com_system_info();
    if(NULL != upper_sys)
    {
        upper_sys->handle_upper_com_flag = 1;
				
        if(5 != upper_sys->socket_status)
        {
            sys->err_num = UPPER_MODULE_ERR;
        }
        sys->upper_work_normal = upper_sys->work_normal;
				
        if(1 == upper_sys->need_read_flag)
        {
            upper_sys->need_read_flag = 0;
            
            num = upper_sys->read_num;
            buf = upper_sys->upper_com_buf;
            i = 0;
            buf_start = 0;
			//ROS_DEBUG("com cmd i:%d,num:%d",i,num);
            while(i < num)
            {
                while((buf[i] != 0x55)&&(i < num))
                {
                    i++;
                }
                if(i == num)
                {
                    break;
                }
                j = (buf[i+PKG_LEN_INDEX+1]<<8)|(buf[i+PKG_LEN_INDEX]);
				//ROS_DEBUG("handle pkg,i:%d,j:%d,:%x",i,j,buf[i+j-1]);
                if((0xAA == buf[i+j-1]) && (i+j <= num) && (j < SOCKET_PKG_LEN) && (j > 6))
                {
                    check_sum = 0;
                    for(k=0;k<j-3;k++)
                    {
                        //ROS_DEBUG("handle pkg,buf[i+%d]:%x,c_tmp:%x\n",k,buf[i+k],c_tmp);
                        check_sum += buf[i+k];
                    }
					check = (buf[i+j-2]<<8)|(buf[i+j-3]);
                    //ROS_DEBUG("handle pkg,c_tmp:%x,buf[j-2]:%x\n",c_tmp,buf[i+j-2]);
                    if(check_sum == check)
                    {
                        handle_cmd(&(buf[i]),sys,motion,env);
                        i += j;    
                        buf_start = i;
                    }
                    else
                    {
                        ROS_DEBUG("handle pkg,check_sum:%x,check:%x",check_sum,check);
                        i++;
                    }
                }
                else
                {
                    i++;
                }
            }
            //ROS_DEBUG("buf_start:%d,num:%d",buf_start,num);
            if(buf_start != num)
            {
                for(k=buf_start;k<num;k++)
                {
                    buf[k-buf_start] = buf[k];
                }
            }
            upper_sys->read_num -= buf_start;
            if(upper_sys->read_num > UPPER_COM_RECV_LEN)
            {
               upper_sys->read_num = 0; 
            }
            //ROS_DEBUG("read_num:%d",upper_com_sys->read_num);
        }
              
        upper_sys->handle_upper_com_flag = 0;       
    }
}


