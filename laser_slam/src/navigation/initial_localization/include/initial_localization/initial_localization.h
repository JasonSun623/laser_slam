#ifndef INITIAL_LOCALIZATION_H_
#define INITIAL_LOCALIZATION_H_

#include <ros/ros.h>


#include <stdio.h>
#include <stdlib.h>

#include "std_msgs/String.h"
#include "json/json.h"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

typedef enum
{
    READY = 0,
    WAIT_GET_CONFIG_FILE,
    WAIT_LOAD_MAP_FINISHED,
    WAIT_SETUP_POSE,
    FINISHED,
}setup_state_e;

class InitialLocalization{
    public:
        InitialLocalization();
        void current_pose_callback(const nav_msgs::Odometry msg);
        void map_list_callback(const std_msgs::String msg);
        bool update_config_file();
        bool read_config_file();
        setup_state_e get_setup_state();
        bool set_state(setup_state_e state);
        bool load_map_pub();
        bool initial_pose_pub();

        nav_msgs::Odometry latest_pose_;
        std::string scene_name_;
        std::string map_name_;
        std::string config_file_path_;

    private:
        setup_state_e initial_state_;
        ros::Subscriber current_pose_sub_;
        ros::Subscriber map_list_sub_;
        ros::Publisher app_pub_;
        ros::Publisher initial_pose_pub_;
        Json::StyledWriter json_writer_;
        YAML::Node config_info;

};

#endif
