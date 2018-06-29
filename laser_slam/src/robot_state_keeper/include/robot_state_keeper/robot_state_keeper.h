/*
* file name:
*/
#ifndef ROBOT_STATE_KEEPER_H_
#define ROBOT_STATE_KEEPER_H_

#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>


#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "json/json.h"
#include "robot_state_keeper/robot_state.h"
#include "robot_state_keeper/RegisterState.h"

#define VERSION_INFO     ("3.1.0.0.22")

#define MSG_PUB_NAME     ("pub_name")
#define MSG_SUB_NAME     ("sub_name")
#define MSG_ERROR_CODE   ("error_code")
#define MSG_MSG          ("msg")
#define MSG_VALUE        ("value")
#define MSG_DATA         ("data")

#define CMD_ROBOT_MODULES_GET         ("robot_modules_get")
#define CMD_ROBOT_VERSION_INFO_GET    ("robot_version_info_get")

using namespace robot_state_keeper;
class RobotStateKeeper {
    public:
        RobotStateKeeper();
        //virtual ~robot_state_keeper();

        //for other modules register
        bool register_robot_modules(robot_state_keeper::RegisterState::Request &req,
                              robot_state_keeper::RegisterState::Response &res);

        //callback to handle cmd_pub topic command json data
        void cmd_callback(const std_msgs::String::ConstPtr& msg);

        //handle json data from cmd_pub topic.
        void cmd_handle(Json::Value &value);

        //timer to send beacon
        void timer_callback(const ros::TimerEvent& e);

        ros::Publisher state_pub;          //publish feedback json data.
        ros::Publisher beacon_pub;
        ros::Subscriber cmd_sub;           //subscribe app command.
        ros::ServiceServer register_srv;   //service server for other modules to register
        ros::Timer beacon_timer;
		
    private:
        std::vector<robot_state> modules;  //save other modules.
        std::string robot_id;
        double beacon_freq;
		
};

#endif
