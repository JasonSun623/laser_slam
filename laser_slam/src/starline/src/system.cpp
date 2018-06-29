#include "ros/ros.h"
#include <sstream>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include "../include/starline/Id.h"
#include "../include/starline/config.h"
#include "../include/starline/system.h"
#include "../include/starline/report.h"
#include "../include/starline/navigation.h"
#include "../include/starline/handle_command.h"
#include "../include/starline/led.h"
#include "../include/starline/sensor.h"  
#include "../include/starline/move.h"
#include "../include/starline/cloud.h"

#define ENTER_EVENT_NUM (10)

void handle_vel(system_t *sys)
{
    double vx = 0.0;
    double vth = 0.0;
    
    if(NULL == sys)
    {
        ROS_DEBUG("system is NULL!");
        return;
    }

    sys->last_vel.vx = sys->real_vel.vx;
    sys->last_vel.vth = sys->real_vel.vth;

    if((1 == sys->base.stop_status) || (1 == sys->event_stop_flag))
    {
        sys->real_vel.vx = 0.0;
        sys->real_vel.vth = 0.0;
        sys->last_vel.vx = 0.0;
        sys->last_vel.vth = 0.0;
    }
    return;
}

void handle_handspike(system_t *sys)
{
	switch(sys->handspike)
	{
		case 0:
			handspike_lift_send_frame();
			break;
		case 1:
			handspike_down_send_frame();
			break;
		case 2:
			handspike_stop_send_frame();
	}
	
}

int read_files(system_t *sys,env_t *env)
{
    int tmp = 0;
    ans_status_t ans;
	
    if((NULL == sys)||(NULL == env))
    {
        ROS_DEBUG("system env is NULL!\n");
        return -1;
    }
    
    if(env->env_mark)
    {
        free(env->env_mark);
		env->env_mark = NULL;
    }
    //tmp = read_mark_file(sys,env);
    if(0 != tmp)
    {
        ROS_DEBUG("read files:read mark file failed!");
		sys->err_num = READ_MARK_FILE_ERR;//to report
		ans.level = LEVEL_INFO;
		ans.module = MODULE_NAV;
		ans.function = 20;
		ans.len = 4;
		set_int_buf(ans.data,3);
		set_event_buffer(&ans);
        return -1;
    }
    
    if(env->env_tpoint)
    {
        free(env->env_tpoint);
		env->env_tpoint = NULL;
    }
    //tmp = read_tpoint_file(sys,env);
    if(0 != tmp)
    {
        ROS_DEBUG("read files:read tpoint file failed!");
		sys->err_num = READ_TPOINT_FILE_ERR;//to report
		ans.level = LEVEL_INFO;
		ans.module = MODULE_NAV;
		ans.function = 20;
		ans.len = 4;
		set_int_buf(ans.data,5);
		set_event_buffer(&ans);
        return -1;
    }

    if(env->env_goal)
    {
        free(env->env_goal);
		env->env_goal = NULL;
    }
    //tmp = read_goal_file(sys,env);
    if(0 != tmp)
    {
        ROS_DEBUG("read files:read goal file failed!");
		sys->err_num = READ_GOAL_FILE_ERR;//to report
		ans.level = LEVEL_INFO;
		ans.module = MODULE_NAV;
		ans.function = 20;
		ans.len = 4;
		set_int_buf(ans.data,7);
		set_event_buffer(&ans);
        return -1;
    }

    if(env->env_map)
    {
        free(env->env_map);
		env->env_map = NULL;
    }
    //tmp = read_map_file(sys,env);
    if(0 != tmp)
    {
        ROS_DEBUG("read files:read map file failed!");
		sys->err_num = READ_MAP_FILE_ERR;//to report
		ans.level = LEVEL_INFO;
		ans.module = MODULE_NAV;
		ans.function = 20;
		ans.len = 4;
		set_int_buf(ans.data,9);
		set_event_buffer(&ans);
        return -1;
    }
    return 0;
}

void init_system_param(system_t *sys,motion_t *motion,env_t *env)
{
    int tmp = 0;

    if((NULL == sys)||(NULL == motion)||(NULL == env))
    {
        ROS_DEBUG("system or motion or env is NULL!\n");
        return;
    }
    memset(sys,0,sizeof(system_t));
    memset(motion,0,sizeof(motion_t));
	memset(env,0,sizeof(env_t));
	
    sys->build_cord_flag = 0;
	sys->need_update_map_flag = 1;	
	sys->auto_enable = MANUAL_MODE; 
	sys->min_z = 1.0;
	sys->max_z = 3.0;
	sys->tolerance_pass = 0.15;
	sys->tolerance_goal = 0.05;
	sys->max_accx = 0.3;//1.2
	sys->max_accth = 0.3;//
	sys->max_vx = 0.35;
	sys->max_vth = 0.45;
	sys->observe_dist = 1.0;
	sys->observe_times = 10;
	sys->control_freq = 20.0;
	sys->max_manual_vx = 0.30;
	sys->max_manual_vth = 0.5;
	sys->video_to_center.x = 0.0;
	sys->video_to_center.y = 0.0;
	sys->video_to_center.th = 0.0;
	sys->rotate_limit = 0.15;
	sys->rotate_pid_p = 0.7;
	sys->line_vx_pid_p = 0.6;
	sys->line_vx_pid_i = 0.0001;
	sys->line_vth_dis_p = 0.03;
	sys->line_vth_ang_p = 0.8;
	sys->final_rotate_pid_p = 1.0;
	sys->dance.dance_xy_range = 0.25;
	sys->dance.dance_xy_scale = 3.0;
	sys->dance.dance_th_range = 0.1;
	//reset_dance_start_point(sys);
	sys->sensor.slow_limit = 0.9;
	sys->sensor.estop_limit = 0.45;
	sys->camera.watch_time = 20;
	set_upper_server_ip((unsigned int)100903104);//"192.168.3.6"
	sys->read_system_file_flag = 0;
	sys->sensor.sonar_event1_limit = 1.0;
	sys->manual_control_over_time = 1.0;
	sys->goal_nearby_range = 0.3;
	sys->enter_event_type = 1;
	sys->enter_event_limit = 1.0;
	sys->nav_over_time = 0.0;
	sys->enter_sonar_num = 4;
	sys->goal_nearby_th = 0.1;
	sys->handspike = 2;
    tmp = read_system_file(sys);
    if(-1 == tmp)
    {
        sys->err_num = READ_SYS_FILE_ERR;
        ROS_DEBUG("read system file failed!");
    }
	else
	{
	    sys->read_system_file_flag = 1;
	}
    ROS_DEBUG("tmp: %d,auto_enable: %d\n",tmp,sys->auto_enable);
    ROS_DEBUG("g_system.min_z: %f,g_system.max_z: %f\n",sys->min_z,sys->max_z);
    ROS_DEBUG("g_system.tolerance_pass: %f\n",sys->tolerance_pass);
    ROS_DEBUG("g_system.tolerance_goal: %f\n",sys->tolerance_goal);
    ROS_DEBUG("g_system.max_accx: %f\n",sys->max_accx);
    ROS_DEBUG("g_system.max_accth: %f\n",sys->max_accth);
    ROS_DEBUG("g_system.max_vx: %f\n",sys->max_vx);
    ROS_DEBUG("g_system.max_vth: %f\n",sys->max_vth);
    ROS_DEBUG("g_system.observe_dist: %f\n",sys->observe_dist);
    ROS_DEBUG("g_system.control_freq: %f\n",sys->control_freq);
    ROS_DEBUG("g_system.rotate_limit: %f\n",sys->rotate_limit);
    ROS_DEBUG("g_system.manual_control_over_time: %f\n",sys->manual_control_over_time);
	ROS_DEBUG("g_system.nav_over_time: %f\n",sys->nav_over_time);
    sys->dance.dance_state = DANCE_FINISHED;
	sys->dance.dance_seq.max_vel.vx = sys->max_vx;
	sys->dance.dance_seq.max_vel.vth = sys->max_vth;
	sys->dance.dance_seq.max_acc_x = sys->max_accx;
	sys->dance.dance_seq.max_acc_th = sys->max_accth;
	sys->camera.state = CAMERA_LOST_CONNECTED;
	sys->starline_download_index = -1;
	sys->base.download_index = -1;
	sys->sensor.download_index = -1;
	sys->led_power.download_index = -1;
    motion->tran.x = 0;
    motion->tran.y = 0;
    motion->tran.th = 0; 
	
    return;
}

void check_sensor_event(system_t *sys)
{
    static int last_sensor_err = 0;
	static int last_sensor_flag = 0;
	static int last_sensor_stop_flag = 0;
    ans_status_t ans;
    int data = 0;
    int i = 0;
	int set_flag = 0;

    if(NULL == sys)
	{
	    return;
	}
		
	//handle sensor error
    set_flag = 0;
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
	if(0 != data)
	{
	    //report event
	    if(last_sensor_err != data)
	    {
	        ans.level = LEVEL_ERROR;
			ans.module = MODULE_SENSOR;
			ans.function = 0;
			ans.len = 4;
			set_int_buf(ans.data,data);
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_sensor_err = data;
	}

	//handle sensor flag
	set_flag = 0;
	data = 0;
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
    if(0 != data)
	{
	    //report event
	    if(last_sensor_flag != data)
	    {
	        ans.level = LEVEL_WARN;
			ans.module = MODULE_SENSOR;
			ans.function = 1;
			ans.len = 5;
			set_int_buf(ans.data,data);
			ans.data[4] = (unsigned char)(sys->sensor.estop_fb_limit*LEN_M_TO_CM);
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_sensor_flag = data;
	}

    //handle stop limit
	set_flag = 0;
	if(0 != sys->sensor.stop_flag)
	{
	    //report event
	    if(last_sensor_stop_flag != sys->sensor.stop_flag)
	    {
	        ans.level = LEVEL_INFO;
			ans.module = MODULE_NAV;
			ans.function = 25;
			ans.len = 4;
			set_int_buf(ans.data,1);
			set_event_buffer(&ans);
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_sensor_stop_flag = sys->base.estop_sensor_flag;
	}

    return;
}

void handle_sensors_info(system_t *sys)
{
    static int last_sensor_com = 0;
	ans_status_t ans;
    int i = 0;
    int stop_flag = 0;
	int slow_flag = 0;

    if(NULL == sys)
    {
        ROS_DEBUG("sys NULL!");
        return;
    }

	//get sensor data from sensor board
    get_sensor_data(sys);
	
    sys->sensor.stop_flag = 0;
	sys->sensor.slow_flag = 0;
    if(2 == sys->sensor.work_normal)
    {		
        //handle obstacle sensors 
        for(i=0;i<LASER_NUM;i++)
        {
            //ROS_DEBUG("laser:%d,len:%f\n",i,system->sensor.laser_len[i]);
            if(sys->sensor.laser_len[i] < sys->sensor.estop_limit)
            {
                stop_flag = 1;
                //ROS_DEBUG("laser[%d]:%f",i,sys->sensor.laser_len[i]);
            }
			else if(sys->sensor.laser_len[i] < sys->sensor.slow_limit)
			{
			    slow_flag = 1;
			}
        }
        for(i=0;i<SONAR_NUM;i++)
        {
            if(sys->sensor.sonar_len[i] < sys->sensor.estop_limit)
            {
                stop_flag = 1;
                //ROS_DEBUG("sonar[%d]:%0.2f,",i,sys->sensor.sonar_len[i]);
            }
			else if(sys->sensor.sonar_len[i] < sys->sensor.slow_limit)
			{
			    slow_flag = 1;
			}
        }
  
    }
    else
    {
        sys->obstacle_scale = OBSTACLE_FREE_SCALE;//gavin-debug
        i = 0;
		if(2 == last_sensor_com)
		{
		    ans.level = LEVEL_ERROR;
			ans.module = MODULE_NAV;
			ans.function = 22;
			ans.len = 4;
			set_int_buf(ans.data,1);
		    i = set_event_buffer(&ans);
		}
    }
	sys->sensor.stop_flag = stop_flag;
	sys->sensor.slow_flag = slow_flag;
	
	//check sensor event
    check_sensor_event(sys);
	if(0 == i)
	{
		last_sensor_com = sys->sensor.work_normal;
	}
    
    return;
}

int check_base_obstacle_stop(system_t *sys)
{
    int obstacle_flag = 0;

    if(NULL == sys)
    {
        return 0;
    }
		
	if(sys->base.estop_sensor_flag & BASE_SENSOR_BUMPER)
    {
        obstacle_flag = 1;
    }
	else if(sys->base.move_status & BASE_STOP_ESTOP)
	{
	    obstacle_flag = 2;
	}
	else if(1 == sys->sensor.stop_flag)
	{
	    obstacle_flag = 3;
	}
	return obstacle_flag;
}

int check_base_error_stop(system_t *sys)
{
    int i = 0;
	int error_flag = 0;

	if(NULL == sys)
	{
	    return 0;
	}

    if(sys->base.estop_sensor_flag & BASE_SENSOR_LASER)
	{
	    error_flag = 1;
	}
	else if(sys->base.move_status & BASE_STOP_ERR)
	{
	    error_flag = 2;
	}
	else if((1==sys->base.power_on_flag)&&(sys->base.move_status&BASE_POWER_ON_BIT))
	{
	    error_flag = 3;
	}
	else 
	{
	    for(i=0;i<BASE_MOTOR_NUM;i++)
	    {
	        if(sys->base.motor_status[i])
	        {
	            error_flag = 4;
	        }
	    }
	}
		
	return error_flag;
}

static double cal_obstacle_scale(int base_stop,int sensor_stop,int sensor_slow)
{
    double scale = OBSTACLE_FREE_SCALE;

	if((1 == base_stop) || (1 == sensor_stop))
	{
	    scale = OBSTACLE_STOP_SCALE;
	}
	else if(1 == sensor_slow)
	{
	    scale = OBSTACLE_SLOW_SCALE;
	}
	else
	{
    	scale = OBSTACLE_FREE_SCALE;
	}
	
    return scale;
}

int check_base_nav(motion_t *motion)
{
    int nav_flag = 0;

    if(NULL == motion)
    {
        return 0;
    }
	/*if(0 == motion->path.path_finish)
    {
        nav_flag = 1;
    }*/
	return nav_flag;
}

static void update_led_effect(system_t *sys,motion_t *motion)
{
	int red_led_flag = 0;
	int yellow_led_flag = 0;
	int nav_led_flag = 0;
	int prior_flag = 0;
	
    if((1 == sys->base.stop_status) || (1 == sys->sensor.stop_flag))
    {
        yellow_led_flag = check_base_obstacle_stop(sys);
		red_led_flag = check_base_error_stop(sys);
        ROS_DEBUG("yellow:%d,red:%d",yellow_led_flag,red_led_flag);
    }
	else
	{
		nav_led_flag = check_base_nav(motion);
	}
	
	set_led_prior(0,red_led_flag);
	set_led_prior(1,yellow_led_flag);
	set_led_prior(2,nav_led_flag);
	
	prior_flag = get_led_prior();
    if(0 != prior_flag)
    {
        if(prior_flag & 0x01)
        {
            if(LED_RED_WARN != sys->led_power.act_effect)
            {
	            set_led_power_effect(LED_POWER_FREEDOM,LED_RED_WARN);
	            ROS_DEBUG("led set to LED_RED_WARN");
            }
        }
		else if(prior_flag & 0x02)
		{
		    if(LED_YELLOE_WARN != sys->led_power.act_effect)
		    {
		        set_led_power_effect(LED_POWER_FREEDOM,LED_YELLOE_WARN);
	            ROS_DEBUG("led set to LED_YELLOW_WARN");
		    }
		}
		else if(prior_flag & 0x04)
		{
		    if(LED_NAV != sys->led_power.act_effect)
			{
				set_led_power_effect(LED_POWER_FREEDOM,LED_NAV);
				ROS_DEBUG("led set to NAV");
			}	
		}
    }
	else
	{
	    if((LED_RED_WARN == sys->led_power.act_effect)
				||(LED_YELLOE_WARN == sys->led_power.act_effect)
				     ||(LED_NAV == sys->led_power.act_effect))
	    {
	        set_led_power_effect(LED_POWER_FREEDOM,LED_NORMAL);
            ROS_DEBUG("led set to LED_NORMAL");
	    }
	}
    return;
}

void check_pushed_status(system_t *sys)
{
    static int pushed_flag[PUSHED_FLAG_OBS_TIMES]={0,};
	static int flag_count = 0;
	int flag = 0;
	int i = 0;
	int j = 0;
	
    if(0 == (sys->base.move_status & 0x10))
    {
		if(0 != (sys->base.move_backup & 0x04))
		{
		    //movebase static
		    if(0 != (sys->base.move_backup & 0x01))
		    {
		        //pushed
		        flag = 1;
				ROS_DEBUG("static pushed");
		    }
			else
			{
			    //isn't pushed
			    flag = 0;
			}
		}
    }

	pushed_flag[flag_count] = flag;
        flag_count = flag_count + 1;
	flag_count = flag_count % PUSHED_FLAG_OBS_TIMES;

	for(i=0;i<PUSHED_FLAG_OBS_TIMES;i++)
	{
	    if(0 == pushed_flag[i])
	    {
	        j++;
	    }
	}
	//ROS_DEBUG("flag_count:%d,j:%d",flag_count,j);
	if(j > (PUSHED_FLAG_OBS_TIMES/2))
	{
	    sys->pushed_flag = 0;
        //ROS_DEBUG("system: does not pushed");
	}
	else
	{
	    sys->pushed_flag = 1;
		//ROS_DEBUG("system:pushed");
	}
	return;
}

void handle_need_modify_pos_event(system_t *sys,motion_t *motion)
{
    static int last_pushed_flag = 0;
	static unsigned char last_estop_key_flag = 0;
	ans_status_t ans;
	unsigned char flag = 0;
	
    //check pushed event
	check_pushed_status(sys);

    if(0 != sys->pushed_flag)
    {
        //report pushed event message
        ans.level = LEVEL_ERROR;
		ans.module = MODULE_NAV;
		ans.function = 23;
		ans.len = 4;
		set_int_buf(ans.data,5);
		set_event_buffer(&ans);

		//control to make movebase stop
		sys->event_stop_flag = 1;
		ROS_DEBUG("pushed:stop robot");
    }
	else
	{
	    
	    if(0 != last_pushed_flag)
	    {
	        //set need update robot position
	        sys->need_modify_pos_flag = 1;
			ROS_DEBUG("pushed:don't push,and need to modify pos");
	    }
	}
    last_pushed_flag = sys->pushed_flag;

	//handle estop key event
	flag = (sys->base.move_status&BASE_ESTOP_KEY_BIT);
	//if is not pushed now,but last pushed,should modify position
	if(0 == flag)
	{
		sys->event_stop_flag = 0;// tom zhang 2017.2.11
	    if(0 != last_estop_key_flag)
	    {
	        sys->need_modify_pos_flag = 1;
			ROS_DEBUG("estop key:release estop key and need to modify pos");
	    }
	}
	else
	{
	    //report estop key message
        ans.level = LEVEL_ERROR;
		ans.module = MODULE_NAV;
		ans.function = 23;
		ans.len = 4;
		set_int_buf(ans.data,7);
		set_event_buffer(&ans);
		
	    sys->event_stop_flag = 1;
		//ROS_DEBUG("estop key:stop robot");
	}
	last_estop_key_flag = flag;

	return;
}

void check_modify_pos_status(system_t *sys)
{
    static int times = MODIFY_POS_OVER_TIMES;
	static int obs_flag = 1;
	ans_status_t ans;

	if((1 == sys->need_modify_pos_flag) && (1 == obs_flag))
	{
	    obs_flag = 0;
	}

	if(0 == obs_flag)
	{
	    times = times -1;
		if(0 == times)
		{
		    obs_flag = 1;
			times = MODIFY_POS_OVER_TIMES;
			if(1 == sys->need_modify_pos_flag)
			{
			    //report modify position failed
			    ans.level = LEVEL_ERROR;
				ans.module = MODULE_NAV;
				ans.function = 23;
				ans.len = 4;
				set_int_buf(ans.data,6);
				set_event_buffer(&ans);
				ROS_DEBUG("report modify position failed");
			}
		}
		if(0 == sys->need_modify_pos_flag)
		{
		    obs_flag = 1;
			times = MODIFY_POS_OVER_TIMES;
		    //report success to modify position
		    ans.level = LEVEL_INFO;
			ans.module = MODULE_NAV;
			ans.function = 25;
			ans.len = 4;
			set_int_buf(ans.data,2);
			set_event_buffer(&ans);
			ROS_DEBUG("report modify position ok");
			//wait modify robot position then could move
			sys->event_stop_flag = 0;
		}
	}
    return;
}

void check_manual_control_overtime(system_t *sys)
{
    static int count = 0;
	int i = 0;

	if(1 == sys->auto_enable)
	{
	    count = 0;
		sys->manual_overtime_flag = 0;
		sys->manual_stop_overtime_flag = 0;
	}
	else
	{
	    count++;
		i = (int)(sys->manual_control_over_time*sys->control_freq); 
		if(i <= 10)
		{
		    i = 10;
		}
		if(0 == (count%i))
		{
		    if(0 != sys->manual_overtime_flag)
			{
				sys->manual_stop_overtime_flag = 1;
			}
			else
			{
			    sys->manual_stop_overtime_flag = 0;
			}
			sys->manual_overtime_flag = 1;
		}
	}
    return;
}

static void check_movebase_stop_event(system_t *sys)
{
    ans_status_t ans;
	int stop_flag = 0;
	int obstacle_flag = 0;


    if((1 == sys->base.stop_status) || (1 == sys->sensor.stop_flag))
	{
	    stop_flag = 1;
	    obstacle_flag = check_base_obstacle_stop(sys);
		ROS_DEBUG("check movebase stop,obstacle flag:%d",obstacle_flag);
	}

    if(0 != stop_flag)
	{
	    if(0 == obstacle_flag)
		{
		    ans.level = LEVEL_ERROR;
			ans.module = MODULE_NAV;
			ans.function = 23;
			ans.len = 4;
			set_int_buf(ans.data,8);
			set_event_buffer(&ans);
		}
		else
		{
		    ans.level = LEVEL_WARN;
			ans.module = MODULE_NAV;
			ans.function = 24;
			ans.len = 4;
			set_int_buf(ans.data,2);
			set_event_buffer(&ans);
		}
	    
	}
	
    return;
}

void handle_system_params(system_t *sys,motion_t *motion,env_t *env)
{
    return;
}

int judge_enter_event_situation(system_t *sys,double cdata[ENTER_EVENT_NUM][4],int num)
{
	static int last_object_in_range = 0;
	double limit = 0.0;
	int enter_trig_flag = 0;
	int exit_trig_flag = 0;
	int object_in_range = 0;
	int event_flag = 0;
	int cur = 0;

    cur = (num+ENTER_EVENT_NUM-1)%ENTER_EVENT_NUM;
	if(sys->enter_event_limit >= 0)
	{
	    limit = sys->enter_event_limit;
	}
	else
	{
	    limit = 2.0;
	}
    if((cdata[cur][0] >= limit) && (cdata[cur][1] >= limit) &&
			(cdata[cur][2] >= limit) && (cdata[cur][3] >= limit))
	{
	    object_in_range = 0;
		
	}
	else
	{
	    object_in_range = 1;
	}
   // ROS_DEBUG("[*]object_in_range:%d,limit:%f",object_in_range,limit);
	if((1 == object_in_range) && (0 == last_object_in_range))
	{
	    if(0 == sys->enter_event_type)
		{
		    //center
	        //ROS_DEBUG("center cdata[%d][*]:%f,%f",cur,cdata[cur][1],cdata[cur][2]);
			if((cdata[cur][2] < limit) || (cdata[cur][1] < limit))
			{
			    event_flag = 2;//get close event
			}
		}
		else if(1 == sys->enter_event_type)
		{
		    //right
	        //ROS_DEBUG("right cdata[%d][*]:%f,%f,%f,%f",cur,cdata[cur][0],
	               // cdata[cur][1],cdata[cur][2],cdata[cur][3]);
			if((cdata[cur][0] < limit) || (cdata[cur][1] < limit))
			{
			    exit_trig_flag = 1;
			}
			else
			{
			    exit_trig_flag = 0;
			}
			if((cdata[cur][2] < limit) || (cdata[cur][3] < limit))
			{
			    enter_trig_flag = 1;
			}
			else
			{
			    enter_trig_flag = 0;
			}
			
		}
		else if(2 == sys->enter_event_type)
		{
		    //left
	        //ROS_DEBUG("left cdata[%d][*]:%f,%f,%f,%f",cur,cdata[cur][0],
	            // cdata[cur][1],cdata[cur][2],cdata[cur][3]);
		    if((cdata[cur][0] < limit) || (cdata[cur][1] < limit))
			{
			    enter_trig_flag = 1;
			}
			else
			{
			    enter_trig_flag = 0;
			}
			if((cdata[cur][2] < limit) || (cdata[cur][3] < limit))
			{
			    exit_trig_flag = 1;
			}
			else
			{
			    exit_trig_flag = 0;
			}
		}
		
		//judge enter or exit event based on enter and exit flag
		//ROS_DEBUG("[*]enter_trig_flag:%d,exit_trig_flag:%d",enter_trig_flag,exit_trig_flag);
        last_object_in_range = object_in_range;
		if((1 == enter_trig_flag) && (0 == exit_trig_flag))
		{
		    event_flag = 1;
		}
		else if((0 == enter_trig_flag) && (1 == exit_trig_flag))
		{
		    event_flag = -1;
		}
	}
	last_object_in_range = object_in_range;
    
    return event_flag;
}	

void check_enter_or_exit_event(system_t *sys, motion_t *motion)
{
    static double data[ENTER_EVENT_NUM][4];
	static int data_count = 0;
	static int data_flag = 0;
	int i = 0;
	ans_status_t ans;

    if(((1 == sys->auto_enable) && (AUTO_BASIC_MODE == sys->auto_work_mode) && (1 == motion->path.path_finish)) 
		|| ((1 == sys->auto_enable) && (AUTO_DANCE_MODE == sys->auto_work_mode) && (DANCE_FINISHED == sys->dance.dance_state)))
	{
        //used center 4 sonar
	    for(i=0;i<4;i++)
	    {
	       data[data_count][i] = sys->sensor.sonar_len[1+i];
	    }
	    data_count = (data_count+1)%ENTER_EVENT_NUM;

	    if((0 == data_count) && (0 == data_flag))
	    {
	        data_flag = 1;
	    }

	    if(0 != data_flag)
	    {
	        //judge min item or min value situation
	        i = judge_enter_event_situation(sys,data,data_count);
		    if(1 == i)
		    {
		        ROS_DEBUG("enter the door");
			    ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 26;
			    ans.len = 4;
			    set_int_buf(ans.data,2);
			    i = set_event_buffer(&ans);
		    }
		    else if(-1 == i)
		    {
		        ROS_DEBUG("exit the door");
			    ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 26;
			    ans.len = 4;
			    set_int_buf(ans.data,3);
			    i = set_event_buffer(&ans);
		    }
		    else if(2 == i)
		    {
		        ROS_DEBUG("get close");
			    ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 26;
			    ans.len = 4;
			    set_int_buf(ans.data,4);
			    i = set_event_buffer(&ans);
		    }
	    }
	}
    return;
}

void check_download_upgrade_file(system_t *sys)
{
	download_t *down = NULL;
    ans_status_t ans;
	if(1 == sys->navigation_upgrade_status)
	{
	    down = get_download_buf(sys->navigation_download_index);
		if(NULL == down)
		{
		    ROS_DEBUG("system check upgrade file failed!");
		    return;
		}

		if(FINISHED == down->download_status)
		{
		    ROS_DEBUG("check_download_upgrade_file navigation result_flag %d",down->result_flag);
            if(0 == down->result_flag)
			{
		    	sys->navigation_upgrade_status = 2;//download finished
		    }
			else
			{
			    //report download failed
			    sys->navigation_upgrade_status = 0;//download failed
			    down->result_flag = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,4);
			    set_int_buf(ans.data+4,MODULE_NAV);
		        set_event_buffer(&ans);
			}
		}
	}

	if(1 == sys->starline_upgrade_status)
	{
	    down = get_download_buf(sys->starline_download_index);
		if(NULL == down)
		{
		    ROS_DEBUG("system check upgrade file failed!");
		    return;
		}

		if(FINISHED == down->download_status)
		{
		    ROS_DEBUG("check_download_upgrade_file starline result_flag %d",down->result_flag);
            if(0 == down->result_flag)
			{
		    	sys->starline_upgrade_status = 2;//download finished
		    }
			else
			{
			    //report download failed
			    sys->starline_upgrade_status = 0;//download failed
			    down->result_flag = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,4);
			    set_int_buf(ans.data+4,MODULE_NAV);
		        set_event_buffer(&ans);
			}
		}
	}

	if(1 == sys->base.upgrade_status)
	{
	    down = get_download_buf(sys->base.download_index);
		if(NULL == down)
		{
		    ROS_DEBUG("system check upgrade file failed!");
		    return;
		}

		if(FINISHED == down->download_status)
		{
		    ROS_DEBUG("check_download_upgrade_file base result_flag %d",down->result_flag);
            if(0 == down->result_flag)
			{
		    	sys->base.upgrade_status = 2;
			}
			else
			{
			    sys->base.upgrade_status = 0;
				down->result_flag = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,4);
			    set_int_buf(ans.data+4,MODULE_BASE);
		        set_event_buffer(&ans);
			}
		}
	}

	if(1 == sys->sensor.upgrade_status)
	{
	    down = get_download_buf(sys->sensor.download_index);
		if(NULL == down)
		{
		    ROS_DEBUG("system check upgrade file failed!");
		    return;
		}

		if(FINISHED == down->download_status)
		{
		    ROS_DEBUG("check_download_upgrade_file sensor result_flag %d",down->result_flag);
            if(0 == down->result_flag)
			{
		    	sys->sensor.upgrade_status = 2;
			}
			else
			{
			    sys->sensor.upgrade_status = 0;
				down->result_flag = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,4);
			    set_int_buf(ans.data+4,MODULE_SENSOR);
		        set_event_buffer(&ans);
			}
		}
	}

	if(1 == sys->led_power.upgrade_status)
	{
	    down = get_download_buf(sys->led_power.download_index);
		if(NULL == down)
		{
		    ROS_DEBUG("system check upgrade file failed!");
		    return;
		}

		if(FINISHED == down->download_status)
		{
		    ROS_DEBUG("check_download_upgrade_file power result_flag %d",down->result_flag);
            if(0 == down->result_flag)
			{
		    	sys->led_power.upgrade_status = 2;
			}
			else
			{
			    sys->led_power.upgrade_status = 0;
				down->result_flag = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,4);
			    set_int_buf(ans.data+4,MODULE_POWER);
		        set_event_buffer(&ans);
			}
		}
	}
    return;
}

int mcu_upgrade_finished(system_t *sys)
{
    int i = 0;
	
    if(NULL == sys)
	{
	    return -1;
	}

	if((0 != sys->base.upgrade_status) 
		|| (0 != sys->base.upgrade_result))
	{
	    i = 1;
	}

	if((0 != sys->sensor.upgrade_status) 
		|| (0 != sys->sensor.upgrade_result))
	{
	    i = 1;
	}

	if((0 != sys->led_power.upgrade_status) 
		|| (0 != sys->led_power.upgrade_result))
	{
	    i = 1;
	}

	return i;
}

int replace_starline_file(system_t *sys)
{
    int i = 0;
	
    if(0 == rename("/home/robot/catkin_ws/install/lib/starline/starline"
		,"/home/robot/catkin_ws/install/lib/starline/starline_last"))
	{
	    i = rename("/home/robot/catkin_ws/download/starline"
			    ,"/home/robot/catkin_ws/install/lib/starline/starline");
		if(0 != i)
		{
		    perror("rename");
			return -1;
		}
	}
	else
	{
	    perror("rename");
		return -1;
	}
    return 0;
}

int tar_navigation_file()
{
    int i = 0;
	i = system("chmod 777 /home/robot/catkin_ws/download/navigation");
	if(0 != i)
	{
	    ROS_DEBUG("handle_cmd:upgrade file chmod file failed");
	    return -1;
	}
	i = system("tar -zxvf /home/robot/catkin_ws/download/navigation -C /home/robot/catkin_ws/");
    if(0 != i)
	{
	    ROS_DEBUG("upgrade tar navigation.tar.gz fail");
	    return -1;
	}
    sync();
    return 0;
}

void decide_and_check_upgrade_system(system_t *sys)
{
    ans_status_t ans;

	static int base_upgrade_count = 0;
	static int sensor_upgrade_count = 0;
	static int led_power_upgrade_count = 0;
	static int mcu_upgrade_count = 0;
    char base_path[128]="/home/robot/catkin_ws/download/base";
    char power_path[128]="/home/robot/catkin_ws/download/power";
    char sensor_path[128]="/home/robot/catkin_ws/download/sensor";
	int i = 0;
	
    if(1 == sys->system_upgrade_flag)
	{
	    if(2 == sys->base.upgrade_status)
		{
		    //set move.cpp to upgrade movebase controller board
		    i = set_movebase_upgrade(base_path,(char *)sys->base.upgrade_md5);
			if(0 == i)
			{
			    sys->base.upgrade_status = 3;
				base_upgrade_count = 0;
			}
		}
		else if(3 == sys->base.upgrade_status)
		{
		    base_upgrade_count++;
			if(0 == get_movebase_upgrade_status())
			{
			    sys->base.upgrade_status = 0;
				sys->base.upgrade_result = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,5);
			    set_int_buf(ans.data+4,MODULE_BASE);
				set_int_buf(ans.data+8,get_movebase_upgrade_result());
		        set_event_buffer(&ans);
			}
			else if(base_upgrade_count > UPGRADE_OVER_TIME)
			{
			    //upgrade failed
			    sys->base.upgrade_status = 0;
				sys->base.upgrade_result = 1;
				//report movebase upgrade failed
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,5);
			    set_int_buf(ans.data+4,MODULE_BASE);
				set_int_buf(ans.data+8,-13);
		        set_event_buffer(&ans);
			}
		}

		if(2 == sys->sensor.upgrade_status)
		{
		    //set sensors.cpp to upgrade movebase controller board
		    i = set_sensor_upgrade(sensor_path,(char *)sys->sensor.upgrade_md5);
			if(0 == i)
			{
			    sys->sensor.upgrade_status = 3;
				sensor_upgrade_count = 0;
			}
		}
		else if(3 == sys->sensor.upgrade_status)
		{
		    sensor_upgrade_count++;
			if(0 == get_sensor_upgrade_status())
			{
			    sys->sensor.upgrade_status = 0;
				sys->sensor.upgrade_result = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,5);
			    set_int_buf(ans.data+4,MODULE_SENSOR);
				set_int_buf(ans.data+8,get_sensor_upgrade_result());
		        set_event_buffer(&ans);
			}
			else if(sensor_upgrade_count > UPGRADE_OVER_TIME)
			{
			    //upgrade failed
			    sys->sensor.upgrade_status = 0;
				sys->sensor.upgrade_result = 1;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,5);
			    set_int_buf(ans.data+4,MODULE_SENSOR);
				set_int_buf(ans.data+8,-9);
		        set_event_buffer(&ans);
			}
		}

		if(2 == sys->led_power.upgrade_status)
		{
		    //set led.cpp to upgrade movebase controller board
		    i = set_power_upgrade(power_path,(char *)sys->led_power.upgrade_md5);
			if(0 == i)
			{
			    sys->led_power.upgrade_status = 3;
				led_power_upgrade_count = 0;
			}
		}
		else if(3 == sys->led_power.upgrade_status)
		{
		    led_power_upgrade_count++;
			if(0 == get_power_upgrade_status())
			{
			    sys->led_power.upgrade_status = 0;
				sys->led_power.upgrade_result = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,5);
			    set_int_buf(ans.data+4,MODULE_POWER);
				set_int_buf(ans.data+8,get_power_upgrade_result());
		        set_event_buffer(&ans);
			}
			else if(led_power_upgrade_count > UPGRADE_OVER_TIME)
			{
			    //upgrade failed
			    sys->led_power.upgrade_status = 0;
				sys->led_power.upgrade_result = 1;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,5);
			    set_int_buf(ans.data+4,MODULE_POWER);
				set_int_buf(ans.data+8,-9);
		        set_event_buffer(&ans);
			}
		}

		//judge base sensor led_power upgrade finish or not
		if(0 == mcu_upgrade_finished(sys))
		{
		    if(2 == sys->navigation_upgrade_status)
			{
			    //replace starline file
			    i = tar_navigation_file();
                ROS_DEBUG("upgrade tar navgation %d",i);
				if(0 == i)
				{
				    ans.level = LEVEL_INFO;
			        ans.module = MODULE_NAV;
			        ans.function = 27;
			        ans.len = 60;
			        set_int_buf(ans.data,5);
			        set_int_buf(ans.data+4,MODULE_NAV);
				    set_int_buf(ans.data+8,0);
		            set_event_buffer(&ans);
                    ROS_DEBUG("upgrade navigation ok!");
				}
				else
				{
				    ans.level = LEVEL_INFO;
			        ans.module = MODULE_NAV;
			        ans.function = 27;
			        ans.len = 60;
			        set_int_buf(ans.data,5);
			        set_int_buf(ans.data+4,MODULE_NAV);
				    set_int_buf(ans.data+8,-1);
		            set_event_buffer(&ans);
				}
				sys->navigation_upgrade_status = 0;
                //report upgrade system finished
                mcu_upgrade_count = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,6);
			    set_int_buf(ans.data+4,0);
		        set_event_buffer(&ans);
				mcu_upgrade_count = 0;
				sys->system_upgrade_flag = 0;
                ROS_DEBUG("decide_and_check_upgrade_system upgrade end");
			}
			//upgrade starline
		    if(2 == sys->starline_upgrade_status)
			{
			    //replace starline file
			    i = replace_starline_file(sys);
                
				if(0 == i)
				{
				    ans.level = LEVEL_INFO;
			        ans.module = MODULE_NAV;
			        ans.function = 27;
			        ans.len = 60;
			        set_int_buf(ans.data,5);
			        set_int_buf(ans.data+4,MODULE_NAV);
				    set_int_buf(ans.data+8,0);
		            set_event_buffer(&ans);
				}
				else
				{
				    ans.level = LEVEL_INFO;
			        ans.module = MODULE_NAV;
			        ans.function = 27;
			        ans.len = 60;
			        set_int_buf(ans.data,5);
			        set_int_buf(ans.data+4,MODULE_NAV);
				    set_int_buf(ans.data+8,-1);
		            set_event_buffer(&ans);
				}
				sys->starline_upgrade_status = 0;
                //report upgrade system finished
                mcu_upgrade_count = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,6);
			    set_int_buf(ans.data+4,0);
		        set_event_buffer(&ans);
				mcu_upgrade_count = 0;
				sys->system_upgrade_flag = 0;
                ROS_DEBUG("decide_and_check_upgrade_system upgrade end");
			}
		}
		else
		{
		    mcu_upgrade_count++;
			if(mcu_upgrade_count > UPGRADE_OVER_TIME)
			{
			    //report system upgrade failed
			    mcu_upgrade_count = 0;
				sys->system_upgrade_flag = 0;
				ans.level = LEVEL_INFO;
			    ans.module = MODULE_NAV;
			    ans.function = 27;
			    ans.len = 60;
			    set_int_buf(ans.data,6);
			    set_int_buf(ans.data+4,1);
		        set_event_buffer(&ans);
			}
		}	
	}
    return;
}

void handle_system_status(system_t *sys,motion_t *motion,env_t *env)
{
    int i = 0;
	int flag = 0;
	ans_status_t ans;
	int sonar_num;	
    if((NULL == sys) || (NULL == env) || (NULL == motion))
	{
	    return;
	}

    //if movebase stop,obstacle_scale should be zero;
    sys->obstacle_scale = cal_obstacle_scale(sys->base.stop_status,sys->sensor.stop_flag
                                               ,sys->sensor.slow_flag);
	
    //handle infrared sensor
    if(1 == sys->sensor.infrared_flag)
    {
        send_pkg_back(PKG_FB_READ_SENSE_DATA,sys,motion,env,1,3,NULL);
    }
		
	//report to pad to shutdown
	if(3 == (sys->led_power.power_status1&0x0f))
	{
		send_pkg_back(PKG_REPORT_SHUTDOWN,sys,motion,env,0,0,NULL);
        ROS_DEBUG("led power need pad shutdown itself");
	}			

    //handle base stop status led
    if(sys->led_power.mmi_test_flag !=1)
    {
        update_led_effect(sys,motion);
    }
	
	//check sonar custom1 event
	flag = 0;
    sonar_num = sys->enter_sonar_num;
    if(sonar_num != 2 && sonar_num != 4 && sonar_num != 6)
    {
        sonar_num = SONAR_NUM;
    }
	for(i=0;i<sonar_num;i++)
    {
        if(sys->sensor.sonar_len[3-sonar_num/2+i] <= sys->sensor.sonar_event1_limit)
        {
            flag = 1;
        }
    }
	if(1 == flag)
	{
	    ans.level = LEVEL_INFO;
		ans.module = MODULE_NAV;
		ans.function = 26;
		ans.len = 4;
		set_int_buf(ans.data,1);
		i = set_event_buffer(&ans);
	}

	//handle need modify position event
	handle_need_modify_pos_event(sys,motion);

	//report modify position status
	check_modify_pos_status(sys);

	//check manual control overtime
	check_manual_control_overtime(sys);

	//check movebase error or obstacle stop
	check_movebase_stop_event(sys);

	//check enter or exit custom event
	check_enter_or_exit_event(sys,motion);

	//check upgrade file download finished or not
	check_download_upgrade_file(sys);

	//decide to upgrade system or not
	decide_and_check_upgrade_system(sys);
    return;
}

void handle_upper_info(motion_t *motion,system_t *sys,env_t *env)
{
    //int itmp = 0;

    if((NULL == sys) || (NULL == motion) || (NULL == env))
    {
        ROS_DEBUG("sys or motion or env NULL!");
        return;
    }
    //notify navigation finished to upper controller
    if(1 == motion->path.info_flag)
    {
        ROS_DEBUG("finish get to goal:%d",motion->path.goal_id);
        send_pkg_back(PKG_REPORT_NAV_FINISHED,sys,motion,env,0,0,NULL);
    }
	//notify dance finished to upper controller
    if(1 == sys->dance.dance_info_flag)
    {
        send_pkg_back(PKG_REPORT_DANCE_FINISHED,sys,motion,env,0,0,NULL);
    }
    
    return;
}

void check_base_event(system_t *sys)
{
	static int last_motor_status = 0;
	static int last_base_status = 0;
	static int last_base_laser_err = 0;
	static int last_base_sensor_flag = 0;
	static int last_power_on_flag = 0;

   	ans_status_t ans;
	int i = 0;
	int data = 0;
	int set_flag = 0;

	if(NULL == sys)
	{
	    return;
	}
	
    //handle sensor flag
	set_flag = 0;
	if(0 != sys->base.estop_sensor_flag)
	{
	    //report event
	    if(last_base_sensor_flag != sys->base.estop_sensor_flag)
	    {
	        ans.level = LEVEL_WARN;
			ans.module = MODULE_BASE;
			ans.function = 3;
			ans.len = 1;
			ans.data[0] = sys->base.estop_sensor_flag;
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_base_sensor_flag = sys->base.estop_sensor_flag;
	}

	//handle laser error
	data = 0;
	for(i = 0;i < BASE_LASER_NUM;i++)
	{
		if(sys->base.laser[i] >= BASE_LASER_ERR_LIMIT)
		{
		    data |= (1<<(4+i));
		}
	}
	set_flag = 0;
	if(0 != data)
	{
	    if(last_base_laser_err != data)
	    {
	        ans.level = LEVEL_ERROR;
			ans.module = MODULE_BASE;
			ans.function = 2;
			ans.len = 1;
		    ans.data[0] = data;
			set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_base_laser_err = data;
	}

	//handle base status
	set_flag = 0;
	data = (sys->base.move_status>>3);
	if(0 != data)
	{
	    //report event
	    if(last_base_status != data)
	    {
	        ans.level = LEVEL_INFO;
			ans.module = MODULE_BASE;
			ans.function = 1;
			ans.len = 1;
			ans.data[0] = data;
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_base_status = data;
	}

	//handle motor status
	set_flag = 0;
	if((sys->base.motor_status[0]&BASE_MOTOR_ERR_BIT) 
				|| (sys->base.motor_status[1]&BASE_MOTOR_ERR_BIT))
	{
	    data = ((int)sys->base.motor_status[0])| (((int)sys->base.motor_status[1])<<8);
	    //report event
	    if(last_motor_status != data)
	    {
	        ans.level = LEVEL_ERROR;
			ans.module = MODULE_BASE;
			ans.function = 0;
			ans.len = 4;
			set_int_buf(ans.data,data);
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_motor_status = data;
	}

	//handle power on flag
	set_flag = 0;
	if(0 == sys->base.power_on_flag)
	{
	    //report event
	    if(last_power_on_flag != sys->base.power_on_flag)
	    {
	        ans.level = LEVEL_ERROR;
			ans.module = MODULE_NAV;
			ans.function = 22;
			ans.len = 4;
			set_int_buf(ans.data,5);
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_power_on_flag = sys->base.power_on_flag;
	}
	return;
}
void handle_movebase(system_t *sys,motion_t *motion)
{
	static int last_base_com = 0;
    move_sys_t *base_sys;
	ans_status_t ans;
	int i = 0;

    if((NULL == sys) || (NULL == motion))
    {
        ROS_DEBUG("sys or motion NULL!");
        return;
    }
		
    base_sys = get_movebase_info();
    if(NULL == base_sys)
    {
        ROS_DEBUG("handle_move_sys is busy now!");
    }
	else if(2 != base_sys->work_normal)
	{
	    sys->base.work_normal = base_sys->work_normal;
		sys->base.move_rssi = base_sys->move_rssi;
	    ROS_DEBUG("movebase does not connected");
		if(2 == last_base_com)
		{
		    ans.level = LEVEL_ERROR;
			ans.module = MODULE_NAV;
			ans.function = 22;
			ans.len = 4;
			set_int_buf(ans.data,2);
		    i = set_event_buffer(&ans);
		}
	}
    else if(0 == base_sys->handle_data_flag)
    {
        base_sys->handle_data_flag = 1;
        
        motion->odom.x = base_sys->odom.x;
        motion->odom.y = base_sys->odom.y;
        motion->odom.th = base_sys->odom.th;
		
	    sys->base.estop_sensor_flag = base_sys->estop_sensor_flag;    
        //0->left,1->right
		for(i=0;i<BASE_MOTOR_NUM;i++)
		{
		    sys->base.motor_status[i] = base_sys->motor_status[i];
		}
		sys->base.move_status = base_sys->move_status;
		sys->base.move_rssi = base_sys->move_rssi;
		sys->base.odom.x = base_sys->odom.x;
        sys->base.odom.y = base_sys->odom.y;
        sys->base.odom.th = base_sys->odom.th;
        sys->base.fb_vel.vx = base_sys->fb_vel.vx;
		sys->base.fb_vel.vy = base_sys->fb_vel.vy;
		sys->base.fb_vel.vth = base_sys->fb_vel.vth;
		sys->base.fb_cmd_vel.vx = base_sys->cmd_vel.vx;
		sys->base.fb_cmd_vel.vy = base_sys->cmd_vel.vy;
		sys->base.fb_cmd_vel.vth = base_sys->cmd_vel.vth;
		sys->base.com_rssi = base_sys->com_rssi;
		sys->base.com_state = base_sys->com_state;
		sys->base.work_normal = base_sys->work_normal;
		sys->base.move_backup = base_sys->move_backup;
		sys->base.move_sensor_state = base_sys->move_sensor_state;
        for(i = 0;i<VERSION_LEN;i++)
        {
            sys->base.version[i] = base_sys->software_version[i];
        }
		for(i = 0;i<BASE_LASER_NUM;i++)
		{
		    sys->base.laser[i] = base_sys->laser[i];
		}
		sys->base.power_v = base_sys->power_v;

		//handle movebase status
		sys->base.stop_status = 0;
        //ROS_DEBUG("movebase status %x",handle_move_sys->move_status);
        if((sys->base.motor_status[0]&BASE_MOTOR_ERR_BIT) 
					|| (sys->base.motor_status[1]&BASE_MOTOR_ERR_BIT))
        {
            ROS_DEBUG("movebase motor error:%x,%x",sys->base.motor_status[0]
							,sys->base.motor_status[1]);
			sys->base.stop_status = 1;
            sys->base.cmd = BASE_CLEAR_ERR_CMD;
        }
        else if(!(sys->base.move_status&BASE_SAFE_MODE_BIT))
        {
            ROS_DEBUG("movebase need to open safe mode");
            sys->base.cmd = BASE_OPEN_SAFE_MODE_CMD;
        }
        else if(sys->base.move_status&BASE_POWER_ON_BIT)
        {
            ROS_DEBUG("movebase clear power on flag");
			if(1 == sys->base.power_on_flag)
			{
			    sys->err_num = BASE_UNNORMAL_POWER_ON_ERR;
				sys->base.stop_status = 1;//base unnormal power on,
				sys->base.cmd = BASE_OPEN_SAFE_MODE_CMD;
			}
			else
			{
            	sys->base.cmd = BASE_CLEAR_POWER_ON_CMD;
			}
        }
		else if(sys->base.move_status&BASE_STOP_BIT)
        {
            //ROS_DEBUG("movebase estop flag:%x,status:%x",sys->base.estop_sensor_flag,sys->base.move_status);
            sys->base.stop_status = 1;
            sys->base.cmd = BASE_OPEN_SAFE_MODE_CMD;
        }
        else
        {
            sys->base.cmd = BASE_NO_CMD;
        }

        if(0 != sys->base.estop_sensor_flag)
        {
            sys->base.stop_status = 1;
        }
				
        //set cmd to handle base error
		base_sys->cmd = (unsigned char)sys->base.cmd;
		
        base_sys->handle_data_flag = 0;

        //handle move base status
		if(!(sys->base.move_status & BASE_POWER_ON_BIT))
		{
		    sys->base.power_on_flag = 1;
		}

        check_base_event(sys);
        
    }
	if(0 == i)
	{
    	last_base_com = sys->base.work_normal;
	}
    return;
}

void check_led_power_event(system_t *sys)
{
    static int last_power_volt = 0;
	static int last_power_recharge = 0;
	static int last_power_current = 0;
	static int last_power_dc = 0;

   	ans_status_t ans;
	int data = 0;
	int set_flag = 0;

	if(NULL == sys)
	{
	    return;
	}
	
    //handle power volt
	set_flag = 0;
	if(sys->led_power.power_status1&0x20)
	{
	    data = sys->led_power.power_status1&0x20;
	    if(last_power_volt != data)
	    {
	        ans.level = LEVEL_WARN;
			ans.module = MODULE_POWER;
			ans.function = 0;
			ans.len = 3;
			ans.data[0] = 1;
			ans.data[1] = sys->led_power.power_v1;
			ans.data[2] = sys->led_power.power_v2;
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_power_volt = data;
	}

	//handle power recharge
	set_flag = 0;
	if(sys->led_power.power_status1&0x10)
	{
	    data = sys->led_power.power_status1&0x10;
	    if(last_power_recharge != data)
	    {
	        ans.level = LEVEL_INFO;
			ans.module = MODULE_POWER;
			ans.function = 1;
			ans.len = 1;
		    ans.data[0] = 1;
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_power_recharge = data;
	}

	//handle power current
	set_flag = 0;
	data = (((int)sys->led_power.err4)<<24)|(((int)sys->led_power.err3)<<16)|(((int)sys->led_power.err2)<<8)|((int)sys->led_power.err1);
	if(0 != data)
	{
	    if(last_power_current != data)
	    {
	        ans.level = LEVEL_WARN;
			ans.module = MODULE_POWER;
			ans.function = 2;
			ans.len = 4;
			set_int_buf(ans.data,data);
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_power_current = data;
	}

	//handle power dc to dc
	set_flag = 0;
	data = ((~sys->led_power.power_switch_status1)>>4)&0x07;
	if(0 != data)
	{
	    if(last_power_dc != data)
	    {
	        ans.level = LEVEL_ERROR;
			ans.module = MODULE_POWER;
			ans.function = 3;
			ans.len = 1;
			ans.data[0] = data;
		    set_flag = set_event_buffer(&ans);
	    }
	}
	if(0 == set_flag)
	{
	    last_power_dc = data;
	}

	return;
}

void check_led_power_info(system_t *sys)
{
    ans_status_t ans;
	int i;
	if(sys->led_power.get_power_status == 1)
	{
		ans.level = LEVEL_INFO;
		ans.module = MODULE_POWER;
		ans.function = 4;
		ans.len = 66;
		for(i=0;i<POWER_CURRENT_NUM*2;i++)
		{
            ans.data[i] = sys->led_power.power_i[i];
		}
		set_event_buffer(&ans);
		clear_get_power_status();
	}
	if(sys->led_power.error_power_status == 1)
   	{
		ans.level = LEVEL_INFO;
		ans.module = MODULE_POWER;
		ans.function = 5;
		ans.len = 3;
		for(i=0;i<ERROR_PASSAGEWAY_DATA_LEN;i++)
		{

            ans.data[i] = sys->led_power.error_power_passageway_data[i];
		}
		set_event_buffer(&ans);
		clear_error_power_status();
    }
}

int handle_led_power(system_t *sys)
{
    static int heart_beat_count = 0;
	static int last_led_power_com = 0;
	ans_status_t ans;
	int i = 0;
	led_power_sys_t *led_power = NULL;
		
    if(NULL == sys)
    {
        return -1;
    }
	
	heart_beat_count++;

    led_power = get_led_power_info();
	if(NULL != led_power)
	{
	    led_power->handle_data_flag = 1;
	    sys->led_power.act_effect  = led_power->fb_effect;
		sys->led_power.act_mode = led_power->fb_mode;
		sys->led_power.com_state = led_power->com_state;
		sys->led_power.sys_status = led_power->sys_status;
		sys->led_power.err1 = led_power->err1;
		sys->led_power.err2 = led_power->err2;
		sys->led_power.err3 = led_power->err3;
		sys->led_power.err4 = led_power->err4;
		sys->led_power.power_status1 = led_power->power_status1;
		sys->led_power.power_status2 = led_power->power_status2;
		sys->led_power.power_switch_status1 = led_power->power_switch_status1;
		sys->led_power.power_switch_status2 = led_power->power_switch_status2;
		sys->led_power.power_switch_status3 = led_power->power_switch_status3;
		sys->led_power.power_switch_status4 = led_power->power_switch_status4;	
		sys->led_power.power_v1 = led_power->power_v1;
		sys->led_power.power_v2 = led_power->power_v2;
		sys->led_power.power_p1 = led_power->power_p1;
		sys->led_power.power_p2 = led_power->power_p2;
		sys->led_power.work_normal = led_power->work_normal;
		sys->led_power.error_power_status = led_power->error_power_status;
		sys->led_power.get_power_status = led_power->get_power_status;
		for(i = 0;i<ERROR_PASSAGEWAY_DATA_LEN;i++)
		{
		    if(i == 0)
		    {
                sys->led_power.error_power_passageway_data[i] = led_power->error_passageway;
		    }
			else
			{
               sys->led_power.error_power_passageway_data[i] = led_power->error_data[i-1];
			}
		}
        for(i = 0;i<VERSION_LEN;i++)
        {
            sys->led_power.version[i] = led_power->software_version[i];
        }
		for(i=0;i<POWER_CURRENT_NUM*2;i++)
		{
		    sys->led_power.power_i[i] = led_power->power_i[i];
		}
		if(0 == (heart_beat_count % LED_POWER_COM_PERIOD))
	    {
	        if(1 == led_power->heart_beat_flag)
	        {
	            sys->led_power.led_sys_status |= 0x01;
	        }
			else
			{
			    sys->led_power.led_sys_status &= (~0x01);
			}
			//set led power board heart beat flag as 1,wait to set to 0 in led thread
			led_power->heart_beat_flag = 1;
	    }
		led_power->handle_data_flag = 0;

        //send event power info or error power data
        check_led_power_info(sys);
		//handle led power event
		check_led_power_event(sys);
		
	}

	i = 0;
	if(2 != sys->led_power.work_normal)
	{
		if(2 == last_led_power_com)
		{
		    ans.level = LEVEL_ERROR;
			ans.module = MODULE_NAV;
			ans.function = 22;
			ans.len = 4;
			set_int_buf(ans.data,6);
		    i = set_event_buffer(&ans);
		}
	}
	if(0 == i)
	{
		last_led_power_com = sys->led_power.work_normal;
	}
	return 0;
}

void init_sys_thread(system_t *sys)
{
    pthread_t sensor_thread;
    pthread_t upper_com_thread;
    pthread_t movebase_thread;
    pthread_t led_thread;
	pthread_t cloud_thread;
    int tmp = 0;

    if(NULL == sys)
    {
        ROS_DEBUG("sys NULL!");
        return;
    }
    
    tmp = pthread_create(&sensor_thread,NULL,sensor_thread_start,NULL); 
    if(0 != tmp)
    {
        ROS_DEBUG("sensor com thread failed!\n");
		sys->err_num = CREATE_THREAD_ERR;
        sys->auto_enable = 0;
    }
    tmp = pthread_create(&upper_com_thread,NULL,upper_com_thread_start,NULL); 
    if(0 != tmp)
    {
        ROS_DEBUG("upper_com thread failed!\n");
		sys->err_num = CREATE_THREAD_ERR;
        sys->auto_enable = 0;
    }
    tmp = pthread_create(&movebase_thread,NULL,movebase_thread_start,NULL); 
    if(0 != tmp)
    {
        ROS_DEBUG("movebase thread failed!\n");
		sys->err_num = CREATE_THREAD_ERR;
        sys->auto_enable = 0;
    }
    tmp = pthread_create(&led_thread,NULL,led_thread_start,NULL); 
    if(0 != tmp)
    {
        ROS_DEBUG("led thread failed!\n");
		sys->err_num = CREATE_THREAD_ERR;
        sys->auto_enable = 0;
    }
    tmp = pthread_create(&cloud_thread,NULL,cloud_com_thread_start,NULL); 
    if(0 != tmp)
    {
        ROS_DEBUG("cloud thread failed!\n");
		sys->err_num = CREATE_THREAD_ERR;
    }
    return;
}

int cmp_equal(int a,int b)
{
    if(a == b)
    {
        return 1;
    }
	else
	{
	    return 0;
	}
}

int cmp_inequal(int a,int b)
{
    if(a != b)
    {
        return 1;
    }
	else
	{
	    return 0;
	}
}

int upgrade_replace_nav_file(void)
{
    int i = 0;
		
    i = system("sudo chmod 777 /home/robot/catkin_ws/src/starline/starline");
	if(0 != i)
	{
	    ROS_DEBUG("handle_cmd:upgrade file chmod file failed");
	    return -1;
	}
	i = system("sudo rm /home/robot/catkin_ws/src/starline/bin/starline");
	//ROS_DEBUG("system rm starline:%d",i);
	if(0 != i)
	{
	    ROS_DEBUG("handle_cmd:upgrade file rm old file failed");
	    return -1;
	}
	i = system("sudo mv /home/robot/catkin_ws/src/starline/starline \
		 /home/robot/catkin_ws/src/starline/bin/");
	//ROS_DEBUG("system mv starline:%d",i);
	if(0 != i)
	{
	    ROS_DEBUG("handle_cmd:upgrade file mv file failed");
	    return -1;
	}
    return 0;
}

int send_pkg(unsigned short int pkg_type,int data,int type,unsigned char * str)
{
    int i = 0;
    i = send_pkg_back(pkg_type,&g_system,&g_motion,&g_env,data,type,str);
	return i;
}

//get version from robot_state_keeper by parameter server
void get_system_version_code(system_t *sys)
{
	// check whether the version num has been stored in sys, if not, get it from robot_state_keeper
	// we count the point number in version code, if the code is not correct, we read it from parameter server.
	// for example the format number : 2.1.0.0.2
	// There are 4 points in the format.
	int point_count = 0;
	if(NULL!=sys->system_version)
	{
		for(unsigned int i = 0; i < VERSION_LEN;++i)
		{
			if('.' == sys->system_version[i])
			{
				point_count++;
			}
		}
	}
	if(VERSION_POINT_NUM!=point_count)
	{	
		std::string version_info; 
		if(ros::param::get("system_version",version_info))
		{
			memcpy(sys->system_version,version_info.c_str(),strlen(version_info.c_str()));
		}
		else
		{
			ROS_DEBUG("Read Version Code Failed");
		}
	}
}

