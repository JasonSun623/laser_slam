#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <boost/thread.hpp>
#include <boost/assign.hpp>

#include "std_msgs/String.h"
#include "ros/ros.h"
#include "json/json.h"
#include "tf/transform_broadcaster.h" 
#include "tf/transform_listener.h"  
#include "tf/message_filter.h"  
#include "tf/tf.h" 
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

#include "localization/localization_sub_params.h"
#include "localization/localization_register.h"
#include "localization/localization_state.h"
#include "robot_state_keeper/robot_state.h"
#include "robot_state_keeper/RegisterState.h"

class LocalizationParameter 
{
    public:
        LocalizationParameter();
        int setParam(std::string name ,std::string value);
        std::string getParam(std::string name);
        // localization yaml path 
        std::string yaml_path;
        std::string amcl_yaml_path;
        std::string amcl_attribute_yaml_path;
        std::string gmapping_yaml_path;
        std::string gmapping_attribute_yaml_path;
        std::string uwb_yaml_path;
        std::string uwb_attribute_yaml_path;
        
    private:
        // save localizaiton state
        std::string localization_state;
        ros::NodeHandle nh;
};

class LocalizationModule
{
    public:
        ~LocalizationModule();
        void setModuleState(std::string name,std::string state);
        // sub module register to localization module
        void registerModule(std::string name);
        // query all module
        std::vector<std::string> queryModule();
        // query nav module
        std::string queryNavModule();
        // query map module
        std::string queryMapModule();
        // get nav module use now
        std::string getCurrentNavModule();
        // get map module use now
        std::string getCurrentMapModule();
        // get default module
        std::string getDefaultModule();
        // set nav module use now
        int setCurrentNavModule(std::string navCurrent);
        // set map module use now
        int setCurrentMapModule(std::string mapCurrent);
        // set localization state now
        int setDefaultModule(std::string defaultModule);
        // is all submodule like amcl, gmapping ok
        bool isModulesOK();
        // get amcl param manager
        SubParamManager getAmclParamManager();
        // get gmapping param manager
        SubParamManager getGmappingParamManager();
        // get gmapping param manager
        SubParamManager getUwbParamManager();
    private:
        // save all sub module state
        std::map<std::string,std::string> module_map;

        LocalizationParameter lp;
};

class LocalizationJson
{
    public:
        // write json str to app
        std_msgs::String writeJsonStr(std::string name, int errorcode, std::string msg, Json::Value data);
        // read json str from app
        bool readJsonStr(const std_msgs::String::ConstPtr& msg);
        // get app pub json data
        Json::Value getAppPubData();
        // get app pub json name
        std::string getAppPubName();
    
    private:
        // app pub json data
        Json::Value app_pub_data;
        // app pub json name
        std::string app_pub_name;
};

class LocalizationNode
{
    public:
        LocalizationNode();
        ~LocalizationNode();

    private:
        // handle app pub topic data
        void appPubCallback(const std_msgs::String::ConstPtr& msg);
        // register localizaiton to system
        void registerSystem();
        // handle registerSystem thread
        void registerSystemThread();
        // get localizaiont state
        bool getState(localization::localization_state::Request &req,
                      localization::localization_state::Response &res);
        // register amcl ok or register gmapping .. ok            
        bool registerLocalizationOK(localization::localization_register::Request &req,
                            localization::localization_register::Response &res);
        // handle app pub command
        void handleCmd(std::string cmdName);
        // handle app sub msg
        void sendMsg(int rlt, std::string cmdName, Json::Value data);

        //use for localization state
        std::string getState();
        int setState(std::string currentState);
             
        // get sub params from vector
        Json::Value getSubParams(std::vector<SubParamValue> subParamVector);
        // get all amcl params
        Json::Value getAmclParams();
        // get all gmapping params
        Json::Value getGmappingParams();
        // pub current pose
        void pubCurrentPose(const nav_msgs::Odometry::ConstPtr& odom_msg);
        // set robot pose
        int setInitialPose(Json::Value revData);
        std::string state;

        ros::NodeHandle nh;
        ros::NodeHandle priv_nh;
        // service for get localization state
        ros::ServiceServer state_service;
        // service for get register state
        ros::ServiceServer register_service;
        ros::Publisher localization_pub;
        ros::Publisher pose_pub;
        ros::Publisher initial_pose_pub;
        ros::Subscriber localization_sub;
        ros::Subscriber odom_sub;

        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        // call system keeper state client
        ros::ServiceClient register_client;
        // register system thread
        boost::thread* register_system_thread;
        //tf listener
        tf::TransformListener* tf_listener;

        // system robot keeper state
        robot_state_keeper::robot_state motion_state;
        LocalizationModule localizationModule;
        LocalizationJson localizationJson;

        double cov_x_1, cov_y_1, cov_th_1, cov_x_2, cov_y_2, cov_th_2;
        int pub_current_pose_count;
};

#endif