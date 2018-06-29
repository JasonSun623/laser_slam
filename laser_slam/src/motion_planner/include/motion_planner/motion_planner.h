/*
* file name:
*/
#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>


#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "json/json.h"
#include "yaml-cpp/yaml.h" //安装yaml-cpp参考google code 主页
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "move_base/Pause.h"
#include "robot_state_keeper/robot_state.h"
#include "robot_state_keeper/RegisterState.h"
#include "tf/transform_broadcaster.h"
#include "motion_planner/motion_planner_state.h"
#include "motion_planner/RegisterPlanner.h"
#include "motion_planner/MotionOp.h"
#include "move_base/ConfigParam.h"
#include <std_msgs/UInt8MultiArray.h>
#include "move_base/FixpathNav.h"
#include "motion_planner/planner_monitor.h"


#define REGISTER_RATE    (0.5)
#define NAV_STATUS_PUB_RTATE    (20)


#define MSG_PUB_NAME     ("pub_name")
#define MSG_SUB_NAME     ("sub_name")
#define MSG_ERROR_CODE   ("error_code")
#define MSG_MSG          ("msg")
#define MSG_VALUE        ("value")
#define MSG_DATA         ("data")

#define PARAM_MAX_VX               ("max_x_vel")
#define PARAM_MAX_VY               ("max_y_vel")
#define PARAM_MAX_VTH              ("max_th_vel")
#define PARAM_DEFAULT_PLANNER      ("default_planner")
#define PARAM_PARAM_NAME           ("param_name")


#define CMD_NAV_GOAL_PAUSE         ("nav_goal_pause")
#define CMD_NAV_GOAL_RESUME        ("nav_goal_resume")
#define CMD_NAV_COSTMAP_PAUSE      ("nav_costmap_pause")
#define CMD_NAV_COSTMAP_RESUME     ("nav_costmap_resume")
#define CMD_SET_PARAM              ("set_motion_planner_param")
#define CMD_GET_PARAM              ("get_motion_planner_param")
#define CMD_NAV_GOAL_START         ("nav_goal_start")
#define CMD_NAV_GOAL_CANCEL        ("nav_goal_cancel")
#define CMD_GET_MOTION_PLANNERS    ("get_motion_planners")
#define CMD_GET_MOTION_STATUS      ("motion_planner_motion_status")
#define CMD_OP_MOVE_BASE_PARAM     ("op_move_base_param")
#define CMD_GET_COSTMAP_PAUSE_STATUS ("get_costmap_update_pause_status")
#define FB_MOTION_PLANNER_EVENT    ("fb_motion_planner_event")


#define CMD_MARKER_GUIDE_START     ("marker_guide_start")
#define CMD_MARKER_GUIDE_CANCEL    ("marker_guide_cancel")
#define CMD_MARKER_GUIDE_PAUSE     ("marker_guide_pause")
#define CMD_MARKER_GUIDE_RESUME    ("marker_guide_resume")

#define CMD_OPEN_MOTION_START     ("open_motion_start")
#define CMD_OPEN_MOTION_CANCEL    ("open_motion_cancel")
#define CMD_OPEN_MOTION_PAUSE     ("open_motion_pause")
#define CMD_OPEN_MOTION_RESUME    ("open_motion_resume")

#define CMD_LANE_FOLLOWING_FORWARD   ("lane_following_forward")
#define CMD_LANE_FOLLOWING_BACKWARD  ("lane_following_backward")
#define CMD_LANE_FOLLOWING_CANCEL    ("lane_following_cancel")
#define CMD_LANE_FOLLOWING_PAUSE     ("lane_following_pause")
#define CMD_LANE_FOLLOWING_RESUME    ("lane_following_resume")

#define CMD_FIXPATH_MOTION_START     ("fixpath_motion_start")
#define CMD_FIXPATH_MOTION_CANCEL    ("fixpath_motion_cancel")
#define CMD_FIXPATH_MOTION_PAUSE     ("fixpath_motion_pause")
#define CMD_FIXPATH_MOTION_RESUME    ("fixpath_motion_resume")


#define MOVE_IDLE                    ("idle")
#define MOVE_BASE_PLANNER            ("move_base")
#define MOVE_MARKER_GUIDE            ("marker_guide")
#define MOVE_OPEN_MOTION             ("open_motion")
#define MOVE_FIXPATH_MOTION          ("fixpath_motion")

#define MOTION_PLANNER_EVENT		 ("motion_planner_event")
#define MOVE_LANE_FOLLOWING_MOTION   ("lane_following_motion")
typedef enum{
    PENDING = 0,
    ACTIVE = 1,
    PREEMPTED = 2,
    SUCCEEDED = 3,
    ABORTED = 4,
    REJECTED = 5,
    PREEMPTING,
    RECALLING,
    RECALLED,
    LOST,
    STUCK = 10,
    PLAN_SUCCESS = 11,
    PLAN_FAILED = 12,
    TMP_FIXPATH_STUCK = 14,
	SOME_ROBOT_ON_THE_PATH = 15
}motion_status_e;

typedef enum{
    NAV_READY = 0,
    NAV_MOVING = 1,
    NAV_PAUSE = 2,
}nav_status_e;

typedef struct
{
    std::string scene;
    std::string current_map;
    std::vector<std::string> map_list;
}map_list_info_t;

//class MotionMethod{
//    public:
//        MotionMethod();

//        virtual motion_pause();
//        virtual motion_resume();
//        virtual motion_start();
//        virtual motion_cancel();
//        virtual bool get_pause_flag();

//    private:
//        bool pause_flag;
//}

//class MoveBaseMethod: class MotionMethod{

//}
class MotionPlanner {
    public:
        MotionPlanner();
        //cmd_pub topic callback function.
        void cmd_callback(const std_msgs::String::ConstPtr& msg);

        //map_list_pub topic callback function.
        void map_list_callback(const std_msgs::String::ConstPtr& msg);

        //handle received json cmd.
        void cmd_handle(const Json::Value &value);

        //control to pause motion
        bool motion_pause(bool pause);

        //control to pause costmap update.
        bool costmap_pause_update(bool pause);

        //initial parameters from config file
        bool init_config_params();

        //service for other motion planners to register
        bool planners_service(motion_planner::RegisterPlanner::Request &req,
                              motion_planner::RegisterPlanner::Response &res);

        bool mark_guide_op(Json::Value value, Json::Value &msg);

        bool open_motion_op(Json::Value value, Json::Value &msg);

        bool fixpath_motion_op(Json::Value value, Json::Value &msg);

        //set parameters and update yaml config file
        Json::Value set_config_param(const Json::Value &value);

        //get parameters
        Json::Value get_config_param(const Json::Value &value);

        //thread to register this module.
        void registerThread();

        //callback to received move_base motion status.
        void movebase_status_callback(const actionlib_msgs::GoalStatusArrayConstPtr& msg);

        bool change_nav_status(nav_status_e status);

		//pub nav status
		void pub_nav_state(u_int8_t state);
		void navStatusPubThread();

        void motion_status_callback(const std_msgs::String::ConstPtr& msg);

        bool lane_following_motion_op(const Json::Value &value, Json::Value &msg);

        bool lane_following_event_handle(const Json::Value &value);


        ros::Publisher state_pub;                       //for publish json feedback
        ros::Subscriber cmd_sub;                        //received json command from app.
        ros::Subscriber movebase_status_sub;              //received move_base motion status.
        ros::Subscriber map_list_sub;                   //received current map list;
        ros::ServiceClient pause_client;                //service client to pause motion
        ros::ServiceClient costmap_pause_update_client; //service client to pause costmap update
        ros::ServiceClient robot_register_client;       //service client to register this module
        ros::ServiceServer planners_srv;                //service server for other motion planners to register
        ros::Publisher cancel_pub;                      //publish to cancel move_base navigation
        ros::Publisher action_goal_pub;                 //publish to start move_base navigation
        ros::ServiceClient configure_move_base_client;  //service client to configure move base param.

        ros::ServiceClient mark_guide_client;
        ros::ServiceClient open_motion_client;
        ros::ServiceClient lane_following_motion_client;
        ros::Subscriber motion_status_sub;
		ros::Publisher nav_status_pub;						//publish nav state
        ros::ServiceClient fixpath_nav_client;
        ros::ServiceClient param_recovery_client;
    private:

        //void cache_mission_cmd(std::string &cmd);

        std::string config_file_path;  //yaml config file path
        YAML::Node config_info;        //for save config data.
        double max_x_vel;
        double max_y_vel;
        double max_th_vel;
        std::string default_planner;
        std::string last_goal_id;
        map_list_info_t map_list_info;
        robot_state_keeper::robot_state motion_state;   //this module register state
        std::vector<motion_planner::motion_planner_state> planners; //for saving all motion planners
        boost::thread* register_thread;                 //for register this module.
		boost::thread* nav_status_thread;                 //fthread for pub nav_status
        u_int8_t last_motion_status;
        geometry_msgs::Pose2D latest_goal;
        std::string latest_goal_map;
        std::string latest_goal_id;
        bool costmap_update_pause_flag;
        nav_status_e nav_status;
        std::string current_nav_task;
        motion_planner::PlannerMonitor planner_recover;
};

#endif
