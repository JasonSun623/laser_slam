#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
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
#include "../include/starline/sensor.h"
#include "../include/starline/move.h"

#include <std_msgs/Int8.h>


system_t g_system;
env_t g_env;
motion_t g_motion;
//int cmd_hs;
FILE *fp = NULL;

void vel_callback(const geometry_msgs::TwistStamped& cmdvel)
{
     g_system.real_vel.vx = cmdvel.twist.linear.x;
     g_system.real_vel.vy = cmdvel.twist.linear.y;
     g_system.real_vel.vth = cmdvel.twist.angular.z;
}

void handspike_callback(std_msgs::Int8 cmd_handspike)
{	ROS_ERROR("g_system.handspike:%d",g_system.handspike);
     //g_system.handspike = 1;
     g_system.handspike = cmd_handspike.data;
	 ROS_INFO("g_system.handspike:%d",g_system.handspike);
     // 0 lift 1 down  2stop 
}


void pub_odom(ros::Publisher odom_pub)
{
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(g_motion.odom.th);
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x = g_motion.odom.x;
    odom.pose.pose.position.y = g_motion.odom.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = g_system.base.fb_vel.vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = g_system.base.fb_vel.vth;

    //publish the message
    odom_pub.publish(odom);
}

void pub_power(ros::Publisher &power_pub )
{
	unsigned char power = 0;
	power = g_system.led_power.power_p1;
	unsigned char status = g_system.led_power.power_status1;
	//std_msgs::Int8 msg;
	//msg.data=power;
	std_msgs::UInt8MultiArray bytes_msg;

	bytes_msg.data.push_back(power);
	bytes_msg.data.push_back(status);
	power_pub.publish(bytes_msg);
}


//20170712,Zero
void pub_baseState(ros::Publisher &basestate_pub )
{

	std_msgs::UInt8MultiArray baseState_msg;

	baseState_msg.data.push_back(baseStateData[0]);
	baseState_msg.data.push_back(baseStateData[1]);
  baseState_msg.data.push_back(baseStateData[2]);
  baseState_msg.data.push_back(baseStateData[3]);
  baseState_msg.data.push_back(baseStateData[4]);
  baseState_msg.data.push_back(baseStateData[5]);
  baseState_msg.data.push_back(baseStateData[6]);
	basestate_pub.publish(baseState_msg);
}

//20170815,Zero,for load motor
void loadMotorCallback(std_msgs::UInt8MultiArray app_data)
{
//  uint8_t cmdData ;
//  cmdData =app_data.data[0];
	loadCMD = app_data.data[0];
	loadFlag = 1;
//  ROS_INFO("cmdData = %d " ,cmdData );
//  loadMotorCMD(cmdData);
}

int main(int argc, char **argv)
{
    geometry_msgs::Twist cmdvel;
    geometry_msgs::PoseStamped curpose;
    int tmp = 0;
    
    ros::init(argc, argv, "starline");
    ros::NodeHandle n;

    ros::Subscriber vel_sub = n.subscribe("cmd_vel",1000,vel_callback);
    ros::Subscriber handspike_sub = n.subscribe("handspike_handle",1000,handspike_callback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom",1000);
    ros::Publisher power_pub = n.advertise<std_msgs::UInt8MultiArray>("power",1);
    tf::TransformBroadcaster odom_broadcaster;

    //20170712,Zero
    ros::Publisher basestate_pub = n.advertise<std_msgs::UInt8MultiArray>("basestate",1);

    //20170815,Zero ,for load motor
    ros::Subscriber loadMotor_sub = n.subscribe("cmd_loadMotor",1,loadMotorCallback);

    /*
     *!!! CLEAR ALL PARAMETERS FIRST  !!!
     */
    init_system_param(&g_system,&g_motion,&g_env);
    init_sys_thread(&g_system);
    init_event_buf();

    ros::NodeHandle nh("base");
    nh.param("pub_base_tf", g_system.pub_base_tf_, 1);  //enable by default

    ros::Rate loop_rate(g_system.control_freq);
    while (ros::ok())
    {

		//get version from robot_state_keeper by parameter server
        get_system_version_code(&g_system);

        //handle movebase data from move
        handle_movebase(&g_system,&g_motion);       

        //get and handle obstacle sensors info
        handle_sensors_info(&g_system);

        //handle led and power board
        handle_led_power(&g_system);
				
        //handle wifi data from pad
        handle_upper_com_cmd(&g_system,&g_motion,&g_env);

        //handle system params
        handle_system_params(&g_system,&g_motion,&g_env);

        //handle system status ,include err
        handle_system_status(&g_system,&g_motion,&g_env);

        //handle vel based acc and mode,then set to movebase
        handle_vel(&g_system);

        //handle_handspike(&g_system);

        set_movebase_cmd_vel(g_system.real_vel);
				
        //set sensors params
        set_sensors_cmd(&g_system);

        //check and report event to upper controller
        handle_report_event();
        
        if(g_system.pub_base_tf_)
        {
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(g_motion.odom.th);
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";
				
            odom_trans.transform.translation.x = g_motion.odom.x;
            odom_trans.transform.translation.y = g_motion.odom.y;
            odom_trans.transform.translation.z = 0;
            odom_trans.transform.rotation = odom_quat;
            odom_broadcaster.sendTransform(odom_trans);
        }
        
        pub_odom(odom_pub);
        pub_power(power_pub);
         //20170712,Zero
        pub_baseState(basestate_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
    fclose(fp);
    ROS_DEBUG("end the application!\n");
    return 0;
}


