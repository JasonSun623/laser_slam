#include "localization/localization.h"

/*
* read json str from  app_pub cmd
*/
bool LocalizationJson::readJsonStr(const std_msgs::String::ConstPtr& msg)
{
   Json::Reader reader;
   Json::Value value;
   if(reader.parse(msg->data,value))
   {
       // pub name is not null and is string type
       if((!value["pub_name"].isNull()) && (value["pub_name"].isString()))
       {
           app_pub_name = value["pub_name"].asString();
           if(!value["data"].isNull())
           {
               app_pub_data = value["data"];
           }
       }      
   }
   return true;
}

/*
* get app_pub data
*/
Json::Value LocalizationJson::getAppPubData()
{
    return app_pub_data;
}

/*
* get app_pub name
*/
std::string LocalizationJson::getAppPubName()
{
   return app_pub_name;
}

/*
* write Json Str with json data value
*/
std_msgs::String LocalizationJson::writeJsonStr(std::string name, int errorcode, std::string msg, Json::Value data)
{
    Json::Value root;
    Json::FastWriter fast;
    std_msgs::String jsonStr;

    root["sub_name"] = Json::Value(name); 
    root["error_code"] = Json::Value(errorcode);
    root["msg"] = Json::Value(msg);
    root["data"] = data;

    jsonStr.data = fast.write(root);
    return jsonStr;
}