#include <iostream>
#include <fstream>

#include "yaml-cpp/yaml.h" 
#include "localization/localization.h"

static const char* LOCALIZATION_YAML = "/home/robot/catkin_ws/src/localization/configs/localization.yaml";
static const char* AMCL_YAML = "/home/robot/catkin_ws/src/localization/configs/amcl_params.yaml";
static const char* AMCL_ATTRIBUTE_YAML = "/home/robot/catkin_ws/src/localization/configs/amcl_params_attribute.yaml";
static const char* GMAPPING_YAML = "/home/robot/catkin_ws/src/localization/configs/gmapping_params.yaml";
static const char* GMAPPING_ATTRIBUTE_YAML = "/home/robot/catkin_ws/src/localization/configs/gmapping_params_attribute.yaml";
static const char* UWB_YAML = "/home/robot/catkin_ws/src/localization/configs/uwb_params.yaml";
static const char* UWB_ATTRIBUTE_YAML = "/home/robot/catkin_ws/src/localization/configs/uwb_params_attribute.yaml";

LocalizationParameter::LocalizationParameter():nh("~")
{
    nh.param("yaml_path", yaml_path,std::string(LOCALIZATION_YAML));
    nh.param("amcl_yaml_path", amcl_yaml_path,std::string(AMCL_YAML));
    nh.param("amcl_attribute_yaml_path", amcl_attribute_yaml_path,std::string(AMCL_ATTRIBUTE_YAML));
    nh.param("gmapping_yaml_path", gmapping_yaml_path,std::string(GMAPPING_YAML));
    nh.param("gmapping_attribute_yaml_path", gmapping_attribute_yaml_path,std::string(GMAPPING_ATTRIBUTE_YAML)); 
    nh.param("uwb_yaml_path", uwb_yaml_path,std::string(UWB_YAML));
    nh.param("uwb_attribute_yaml_path", uwb_attribute_yaml_path,std::string(UWB_ATTRIBUTE_YAML));
}

/*
* get param from localization.yaml
*/
std::string LocalizationParameter::getParam(std::string name)
{
    std::string rlt;
    std::ifstream fin(yaml_path.c_str());

    // open fail
    if(!fin)
    {
        ROS_ERROR("fail open localization yaml");
        return rlt;
    }
    YAML::Node doc = YAML::Load(fin);

    try
    {
       rlt = doc[name].as<std::string>();
       //ROS_INFO("localization %s:%s",name.c_str(),rlt.c_str());
    }
    catch(YAML::Exception)
    {
       ROS_ERROR("LocalizationParameter get invalid tag");
    }
    fin.close();
    return rlt;
}

/*
* set param(name,value) to localization.yaml
*/
int LocalizationParameter::setParam(std::string name ,std::string value)
{
    std::ifstream fin(yaml_path.c_str());
    int rlt = 0;
    // open fin fail
    if(!fin)
    {
        ROS_ERROR("fail open localization yaml");
        return rlt;
    }

    YAML::Node doc = YAML::Load(fin);
    fin.close();
    std::ofstream fout(yaml_path.c_str());
    // open fout fail
    if(!fout)
    {
        ROS_ERROR("fail open localization yaml");
        return rlt;
    }

    try
    {
       doc[name] = value;
       fout << doc;
    }
    catch(YAML::Exception)
    {
       ROS_ERROR("LocalizationParameter set invalid tag");
       rlt = 1;
    }
    fout.close();
    return rlt;
}
