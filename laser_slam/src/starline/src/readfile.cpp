#include <sstream>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../include/starline/config.h"
#include "../include/starline/system.h"

int read_system_file(system_t *sys)
{
    int i=0;
	int item_count = 0;
    FILE *f;
    double value = 0.0;
    char string[64];

    if(NULL == sys)
    {
        ROS_DEBUG("sys NULL!");
        return -1;
    }

    f=fopen("/home/robot/catkin_ws/install/share/starline/cfgfile/system.cfg","r");
    if(NULL == f)
    {
        ROS_DEBUG("open system.cfg file failed!\n");
        return -1;
    }
    printf("open system.cfg file ok\n");
    while(!feof(f))
    {
        i = fscanf(f,"%lf,%s",&value,string);
        printf("fscanf system.cfg file :%d,string:%s\n",i,string);
        if(i < 0)
        {
            continue;
        }
        if((0 == strcmp(string,"AUTO_ENABLE")) && (value >= 0))
        {
            sys->auto_enable = value;
			item_count++;
        }    
        else if((0 == strcmp(string,"MIN_Z")) && (value > 0))
        {
            sys->min_z = value;
			item_count++;
        }
        else if((0 == strcmp(string,"MAX_Z")) && (value > 0))
        {
            sys->max_z = value;
			item_count++;
        }
        else if((0 == strcmp(string,"TOLERANCE_PASS")) && (value > 0))
        {
            sys->tolerance_pass = value;
			item_count++;
        }
        else if((0 == strcmp(string,"TOLERANCE_GOAL")) && (value > 0))
        {
            sys->tolerance_goal = value;
			item_count++;
        }
        else if((0 == strcmp(string,"MAX_ACC")) && (value > 0))
        {
            sys->max_accx = value;
			item_count++;
        }
        else if((0 == strcmp(string,"MAX_ACCTH")) && (value > 0))
        {
            sys->max_accth = value;
			item_count++;
        }
        else if((0 == strcmp(string,"MAX_VX")) && (value > 0))
        {
            sys->max_vx = value;
			item_count++;
        }
        else if((0 == strcmp(string,"MAX_VTH")) && (value > 0))
        {
            sys->max_vth = value;
			item_count++;
        }
        else if((0 == strcmp(string,"OBSERVE_DIST")) && (value > 0))
        {
            sys->observe_dist = value;
			item_count++;
        }
        else if((0 == strcmp(string,"OBSERVE_TIMES")) && (value > 0))
        {
            sys->observe_times= value;
			item_count++;
        }
        else if((0 == strcmp(string,"CONTROL_FREQ")) && (value > 0))
        {
            sys->control_freq = value;
			item_count++;
        }
        else if((0 == strcmp(string,"ESTOP_LIMIT")) && (value >= 0))
        {
            sys->sensor.estop_limit = value;
			item_count++;
        }
        else if((0 == strcmp(string,"SLOW_LIMIT")) && (value >= 0))
        {
            sys->sensor.slow_limit = value;
			item_count++;
        }
        else if((0 == strcmp(string,"MAX_MANUAL_VX")) && (value > 0))
        {
            sys->max_manual_vx = value;
			item_count++;
        }
        else if((0 == strcmp(string,"MAX_MANUAL_VTH")) && (value > 0))
        {
            sys->max_manual_vth = value;
			item_count++;
        }
        else if((0 == strcmp(string,"VIDEO_TO_CENTER_X")) )
        {
            sys->video_to_center.x = value;
			item_count++;
        }
        else if((0 == strcmp(string,"VIDEO_TO_CENTER_Y")) )
        {
            sys->video_to_center.y = value;
			item_count++;
        }
        else if((0 == strcmp(string,"VIDEO_TO_CENTER_TH")) )
        {
            sys->video_to_center.th = value;
			item_count++;
        }
        else if((0 == strcmp(string,"ROTATE_PID_P")) && (value > 0))
        {
            sys->rotate_pid_p = value;
			item_count++;
        }
        else if((0 == strcmp(string,"ROTATE_LIMIT")) && (value > 0))
        {
            sys->rotate_limit = value;
			item_count++;
        }
        else if((0 == strcmp(string,"LINE_VX_PID_P")) && (value > 0))
        {
            sys->line_vx_pid_p = value;
			item_count++;
        }
        else if((0 == strcmp(string,"LINE_VX_PID_I")) && (value >= 0))
        {
            sys->line_vx_pid_i = value;
			item_count++;
        }
        else if((0 == strcmp(string,"LINE_VTH_DIS_P")) && (value > 0))
        {
            sys->line_vth_dis_p = value;
			item_count++;
        }
        else if((0 == strcmp(string,"LINE_VTH_ANG_P")) && (value > 0))
        {
            sys->line_vth_ang_p = value;
			item_count++;
        }
        else if((0 == strcmp(string,"FINAL_ROTATE_PID_P")) && (value > 0))
        {
            sys->final_rotate_pid_p = value;
			item_count++;
        }
        else if((0 == strcmp(string,"DANCE_XY_RANGE")) && (value > 0))
        {
            sys->dance.dance_xy_range = value;
			item_count++;
        }
        else if((0 == strcmp(string,"DANCE_TH_RANGE")) && (value > 0))
        {
            sys->dance.dance_th_range = value;
			item_count++;
        }
        else if((0 == strcmp(string,"DANCE_XY_SCALE")) && (value > 0))
        {
            sys->dance.dance_xy_scale = value;
			item_count++;
        }
        else if(0 == strcmp(string,"DANCE_START_X"))
        {
            sys->dance.dance_start_point.x = value;
			item_count++;
        }
        else if(0 == strcmp(string,"DANCE_START_Y"))
        {
            sys->dance.dance_start_point.y = value;
			item_count++;
        }
        else if(0 == strcmp(string,"DANCE_START_TH"))
        {
            sys->dance.dance_start_point.th = value;
			item_count++;
        }
        else if((0 == strcmp(string,"DANCE_START_SET")) && (value >= 0))
        {
            sys->dance.dance_start_point_set = value;
			item_count++;
        }
		else if((0 == strcmp(string,"CAMERA_WATCH_TIME")) && (value > 0))
        {
            sys->camera.watch_time = value;
			item_count++;
        }
		else if(0 == strcmp(string,"SERVER_IP"))
        {
			set_upper_server_ip((unsigned int)value);
			item_count++;
        }
		else if(0 == strcmp(string,"SONAR_EVENT_A"))
        {
            sys->sensor.sonar_event1_limit = value;
			item_count++;
        }
		else if((0 == strcmp(string,"MANUAL_CONTROL_OVER_TIME")) && (value > 0))
        {
            sys->manual_control_over_time = value;
			item_count++;
        }
		else if(0 == strcmp(string,"ENTER_EVENT_TYPE"))
        {
            sys->enter_event_type = value;
			item_count++;
        }		
		else if((0 == strcmp(string,"ENTER_EVENT_LIMIT")) && (value >= 0))
        {
            sys->enter_event_limit = value;
			item_count++;
        }
		else if((0 == strcmp(string,"GOAL_NEARBY_RANGE")) && (value >= 0))
        {
            sys->goal_nearby_range = value;
			item_count++;
        }
		else if((0 == strcmp(string,"NAV_OVER_TIME")) && (value >= 0))
        {
            sys->nav_over_time = value;
			item_count++;
        }
		else if(0 == strcmp(string,"ENTER_SONAR_NUM"))
        {
            sys->enter_sonar_num = value;
			item_count++;
        }
		else if((0 == strcmp(string,"GOAL_NEARBY_TH")) && (value >= 0))
        {
            sys->goal_nearby_th = value;
			item_count++;
        }
                else if(0 == strcmp(string,"RESERVE_INT_1"))
        {
            sys->reserve_int_1 = value;
			item_count++;
        }
		else if(0 == strcmp(string,"RESERVE_INT_2"))
        {
            sys->reserve_int_2 = value;
			item_count++;
        }
		else if(0 == strcmp(string,"RESERVE_DOUBLE_3"))
        {
            sys->reserve_double_3 = value;
			item_count++;
        }
		else if(0 == strcmp(string,"RESERVE_DOUBLE_4"))
        {
            sys->reserve_double_4 = value;
			item_count++;
        }
        else
        {
            printf("read system error\n");
            return -1;
        }

    }
    fclose(f);

    if(SYSTEM_CFG_ITEM_NUM != item_count)
	{
	    //return -1;
	}
    return 0;
}

int write_system_file(system_t *sys)
{
    FILE *f;

    if(NULL == sys)
    {
        ROS_DEBUG("write_system_file\uff1a sys NULL!");
        return -1;
    }

    f=fopen("/home/robot/catkin_ws/install/share/starline/cfgfile/system.cfg","w");
    if(NULL == f)
    {
        ROS_DEBUG("write system.cfg open failed!\n");
        return -1;
    }
    printf("write system.cfg open ok\n");

    fprintf(f,"%d,%s\n",sys->auto_enable,"AUTO_ENABLE");
    fprintf(f,"%.3f,%s\n",sys->min_z,"MIN_Z");
    fprintf(f,"%.3f,%s\n",sys->max_z,"MAX_Z");
    fprintf(f,"%.3f,%s\n",sys->tolerance_pass,"TOLERANCE_PASS");
    fprintf(f,"%.3f,%s\n",sys->tolerance_goal,"TOLERANCE_GOAL");
    fprintf(f,"%.3f,%s\n",sys->max_accx,"MAX_ACC");
    fprintf(f,"%.3f,%s\n",sys->max_accth,"MAX_ACCTH");
    fprintf(f,"%.3f,%s\n",sys->max_vx,"MAX_VX");
    fprintf(f,"%.3f,%s\n",sys->max_vth,"MAX_VTH");
    fprintf(f,"%.3f,%s\n",sys->observe_dist,"OBSERVE_DIST");
    fprintf(f,"%d,%s\n",sys->observe_times,"OBSERVE_TIMES");
    fprintf(f,"%.3f,%s\n",sys->control_freq,"CONTROL_FREQ");
    fprintf(f,"%.3f,%s\n",sys->sensor.estop_limit,"ESTOP_LIMIT");
    fprintf(f,"%.3f,%s\n",sys->sensor.slow_limit,"SLOW_LIMIT");
    fprintf(f,"%.3f,%s\n",sys->max_manual_vx,"MAX_MANUAL_VX");
    fprintf(f,"%.3f,%s\n",sys->max_manual_vth,"MAX_MANUAL_VTH");
    fprintf(f,"%.3f,%s\n",sys->video_to_center.x,"VIDEO_TO_CENTER_X");
    fprintf(f,"%.3f,%s\n",sys->video_to_center.y,"VIDEO_TO_CENTER_Y");
    fprintf(f,"%.3f,%s\n",sys->video_to_center.th,"VIDEO_TO_CENTER_TH");
    fprintf(f,"%.3f,%s\n",sys->rotate_pid_p,"ROTATE_PID_P");
    fprintf(f,"%.3f,%s\n",sys->rotate_limit,"ROTATE_LIMIT");
    fprintf(f,"%.3f,%s\n",sys->line_vx_pid_p,"LINE_VX_PID_P");
    fprintf(f,"%.4f,%s\n",sys->line_vx_pid_i,"LINE_VX_PID_I");
    fprintf(f,"%.3f,%s\n",sys->line_vth_dis_p,"LINE_VTH_DIS_P");
    fprintf(f,"%.3f,%s\n",sys->line_vth_ang_p,"LINE_VTH_ANG_P");
    fprintf(f,"%.3f,%s\n",sys->final_rotate_pid_p,"FINAL_ROTATE_PID_P");
    fprintf(f,"%.3f,%s\n",sys->dance.dance_xy_range,"DANCE_XY_RANGE");
    fprintf(f,"%.3f,%s\n",sys->dance.dance_xy_scale,"DANCE_XY_SCALE");
    fprintf(f,"%.3f,%s\n",sys->dance.dance_th_range,"DANCE_TH_RANGE");
    fprintf(f,"%.3f,%s\n",sys->dance.dance_start_point.x,"DANCE_START_X");
    fprintf(f,"%.3f,%s\n",sys->dance.dance_start_point.y,"DANCE_START_Y");
    fprintf(f,"%.3f,%s\n",sys->dance.dance_start_point.th,"DANCE_START_TH");
    fprintf(f,"%d,%s\n",sys->dance.dance_start_point_set,"DANCE_START_SET");
	fprintf(f,"%d,%s\n",sys->camera.watch_time,"CAMERA_WATCH_TIME");
    fprintf(f,"%u,%s\n",get_upper_server_ip(),"SERVER_IP");
	fprintf(f,"%.3f,%s\n",sys->sensor.sonar_event1_limit,"SONAR_EVENT_A");
	fprintf(f,"%.3f,%s\n",sys->manual_control_over_time,"MANUAL_CONTROL_OVER_TIME");
	fprintf(f,"%d,%s\n",sys->enter_event_type,"ENTER_EVENT_TYPE");
	fprintf(f,"%.3f,%s\n",sys->enter_event_limit,"ENTER_EVENT_LIMIT");
	fprintf(f,"%.3f,%s\n",sys->goal_nearby_range,"GOAL_NEARBY_RANGE");
	fprintf(f,"%.3f,%s\n",sys->nav_over_time,"NAV_OVER_TIME");
	fprintf(f,"%d,%s\n",sys->enter_sonar_num,"ENTER_SONAR_NUM");
	fprintf(f,"%.3f,%s\n",sys->goal_nearby_th,"GOAL_NEARBY_TH");
	fprintf(f,"%d,%s\n",sys->reserve_int_1,"RESERVE_INT_1");
	fprintf(f,"%d,%s\n",sys->reserve_int_2,"RESERVE_INT_2");
	fprintf(f,"%.3f,%s\n",sys->reserve_double_3,"RESERVE_DOUBLE_3");
	fprintf(f,"%.3f,%s\n",sys->reserve_double_4,"RESERVE_DOUBLE_4");

    printf("system.cfg write done!\n");
    fclose(f);
    return 0;
}


int read_led_file(system_t *sys,char *path)
{
    if((NULL == sys) || (NULL == path))
    {
        ROS_ERROR("sys or path is NULL");
		return -1;
    }
	
    return 0;
}


