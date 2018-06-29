#ifndef OPEN_MOTION_PLANNER_H_
#define OPEN_MOTION_PLANNER_H_


#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>


#include "ros/ros.h"
#include "ros/console.h"
#include "ros/master.h"
#include "std_msgs/String.h"
#include "ros/time.h"
#include "ros/connection.h"
#include "json/json.h"
#include "yaml-cpp/yaml.h" //安装yaml-cpp参考google code 主页
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "motion_planner/MotionOp.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"


#define MSG_PUB_NAME     ("pub_name")
#define MSG_SUB_NAME     ("sub_name")
#define MSG_ERROR_CODE   ("error_code")
#define MSG_MSG          ("msg")
#define MSG_VALUE        ("value")
#define MSG_DATA         ("data")


#define EQUAL_VALUE      (0.001)

#define CMD_OPEN_MOTION_START     ("open_motion_start")
#define CMD_OPEN_MOTION_CANCEL    ("open_motion_cancel")
#define CMD_OPEN_MOTION_PAUSE     ("open_motion_pause")
#define CMD_OPEN_MOTION_RESUME    ("open_motion_resume")

#define MOVE_OPEN_MOTION             ("open_motion")

typedef enum{
    NAV_READY = 0,
    NAV_MOVING = 1,
    NAV_PAUSE = 2,
}nav_status_e;

typedef enum{
    VEL_MODE = 0,
    RELATIVE_POS_MODE = 1,
    ABSOLUTE_POS_MODE = 2,
}motion_type_e;

/*
 * 1 vel+ timer.
 * 2 x,y,th relative target + time.
 * 3 x,y,th absolute move.
 *
*/

typedef struct move_action{
    double vx;
    double vy;
    double vth;

    double target_x;
    double target_y;
    double target_th;

    ros::Time start_time;
    double time;

    motion_type_e motion_type;
    nav_status_e status;
    bool update_info;
    bool ignore_xy_range;
    bool finished_flag;
    geometry_msgs::Pose2D start_pose;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Pose2D current_pose;
}move_action_t;

typedef struct move_param{
    double min_vx;
    double min_vy;
    double min_vth;
    double max_vx;
    double max_vy;
    double max_vth;
    double max_acc_x;
    double max_acc_y;
    double max_acc_th;
}move_param_t;


class OpenMotionPlanner{
    public:
        OpenMotionPlanner();
        ~OpenMotionPlanner();
        void PlanVelThread();
        void RobotCurrentPose(const nav_msgs::Odometry::ConstPtr &msg);
        double CheckVelMinAndMaxLimit(double v,double min_v,double max_v);
        bool MotionOp(motion_planner::MotionOp::Request &req,motion_planner::MotionOp::Response &res);
        Json::Value HandleCmd(Json::Value cmd);
        bool CalRelativeMotion(move_action_t& action,move_param_t& param);
        bool CalAbsoluteMotion(move_action_t& action,move_param_t& param);
        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void check_obstacle_vel(geometry_msgs::Twist &twist);


    private:
        ros::Publisher vel_pub_;
        ros::Publisher motion_status_pub_;
        ros::Subscriber robot_pose_sub_;
        ros::Subscriber scan_sub_;
        ros::ServiceServer finish_srv_;
        ros::ServiceServer op_srv;
        tf::TransformListener listener;
        boost::thread* vel_thread_;
        bool motion_finish_flag_;           //subscribe to get robot move backward enough to trig hall key
        bool pause_;                        //pause flag
        bool guiding_;
        nav_status_e status_;
        move_param_t motion_param_;
        move_action_t action_;
        nav_msgs::Odometry robot_pose_;
        sensor_msgs::LaserScan latest_scan_;
        double stop_limit_;

};

#endif
