#ifndef LOCALIZATION_SUBMODULE_PARAMS_H
#define LOCALIZATION_SUBMODULE_PARAMS_H

#include "ros/ros.h"
#include "yaml-cpp/yaml.h" 
#include "std_msgs/String.h"
#include "json/json.h"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>

/*
* app set yaml vaule result
*/
typedef enum {
    SET_SUCCESS = 0,
    EMPTY_ERROR = 1,
    TYPE_ERROR = 2,
    RANGE_ERROR = 3,
    OPEN_ERROR = 4,
    SET_ERROR = 5,
    OTHER_ERROR = 6,
}setValueRlt;

/*
* sub module param attribute 
* e.g. -{name:xx,type:int,max:xx,min:xx,default:xx}
*/
struct SubParamAttribute
{
     std::string name;
     std::string type;
     boost::any max;
     boost::any min;
     boost::any defaultValue;
};

/*
* used for app query param now
*/
struct SubParamValue
{
     std::string name;
     std::string type;
     boost::any value;
};

class SubParamManager
{
     public:
         SubParamManager(std::string paramAttiYaml,std::string paramYaml);
         // app get sub module param list
         std::vector<SubParamValue> getSubParamsVector();
         // app set sub module param
         int setSubParamValue(std::string name, Json::Value jsonValue);
         
     private:
         // get sub module param attribute list from yaml in init
         void getParamAttiVector();
         // get sub module param list in init
         void getParamValueVector();
         // get param attribute for name
         SubParamAttribute getParamAtti(std::string name);
         // get param vaule for name in param yaml
         boost::any getParamValue(std::string name);
         // check set value is right
         int checkValue(std::string name,Json::Value jsonValue, boost::any& value);
         // trans json value into real type
         int transParamType(std::string name, Json::Value jsonValue, boost::any& value);
         // is set value is in range
         bool isParamInRange(std::string name, boost::any value);
         
         std::string paramAttiYaml;
         std::string paramYaml;
         //sub module param attribute list
         std::vector<SubParamAttribute> subParamsAttiVector;
         //sub module param list
         std::vector<SubParamValue> subParamsValueVector;                
};

#endif