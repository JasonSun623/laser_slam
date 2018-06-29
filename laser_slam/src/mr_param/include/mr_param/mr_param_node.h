#ifndef MR_PARAM_H
#define MR_PARAM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/any.hpp>
#include <boost/algorithm/string.hpp>

#include "std_msgs/String.h"
#include "ros/ros.h"
#include "json/json.h"
#include "yaml-cpp/yaml.h"

struct ParamAttribute
{
     std::string name;
     std::string type;
     boost::any defaultValue;
};

class MrParamYaml
{
    public:
        MrParamYaml();
        ~MrParamYaml();

        std::string getPkgsList();
        std::string getPkgParams(std::string pkgName);
        bool setPkgParams(std::string pkgName,std::string params);
        bool restorePkgParams(std::string pkgName);
    
    private:
        std::string getData(std::string path,std::string data_name);
        std::string getData(std::string path,std::vector<ParamAttribute> data_attribute);
        std::vector<ParamAttribute> getAttributeVector(std::string path);
        std::map<std::string,std::string> getParamType(std::string path);
        bool setParams(std::string path,std::vector<std::string> params,
                        std::map<std::string,std::string> param_type);
        std::vector<std::string> getParamDefault(std::string path);

        ros::NodeHandle nh;
        ros::NodeHandle priv_nh;
        std::string config_path;
};

class MrParamNode
{
    public:
        MrParamNode();
        ~MrParamNode();

    private:
        bool readJsonStr(const std_msgs::String::ConstPtr& msg);
        std_msgs::String writeJsonStr(std::string name, int errorcode, std::string msg, Json::Value data);
        void sendMsg(int rlt, std::string cmdName, Json::Value data);
        void appPubCallback(const std_msgs::String::ConstPtr& msg);
        
        ros::NodeHandle nh;
        Json::Value app_pub_data;
        std::string app_pub_name;
        ros::Publisher mr_param_pub;
        ros::Subscriber mr_param_sub;

        MrParamYaml mrParamYaml;
};

#endif