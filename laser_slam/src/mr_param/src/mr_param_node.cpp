#include "mr_param/mr_param_node.h"

static const int ERRORCODE_SUCCESS = 0;
static const int ERRORCODE_FAIL = 1;

MrParamNode::MrParamNode()
{
    mr_param_sub = nh.subscribe("app_pub", 1, &MrParamNode::appPubCallback,this);
    mr_param_pub = nh.advertise<std_msgs::String>("app_sub", 1);
}

MrParamNode::~MrParamNode()
{
}

void MrParamNode::appPubCallback(const std_msgs::String::ConstPtr& msg)
{     
    Json::Value sendData;
    int rlt;
    std::string rltStr;
    bool sendflag = false;
    std::string pkgName;
    std::string params;

    if(readJsonStr(msg))
    {
        if(0 == app_pub_name.compare("pkgs_list_get"))
        {            
            rltStr = mrParamYaml.getPkgsList();
            rlt = (0 != rltStr.compare(""))?ERRORCODE_SUCCESS:ERRORCODE_FAIL;
            sendData["pkgs"] = rltStr;
            sendflag = true;
        }
        else if(0 == app_pub_name.compare("pkg_params_get"))
        {            
            if(!app_pub_data.isNull() && app_pub_data["pkg_name"].isString())
            {
                pkgName = app_pub_data["pkg_name"].asString();
                rltStr = mrParamYaml.getPkgParams(pkgName);
                rlt = (0 != rltStr.compare(""))?ERRORCODE_SUCCESS:ERRORCODE_FAIL;
                sendData["params"] = rltStr;
                sendflag = true;
            }
        }
        else if(0 == app_pub_name.compare("pkg_params_set"))
        {            
            if(!app_pub_data.isNull() && app_pub_data["pkg_name"].isString() && app_pub_data["params"].isString())
            {
                pkgName = app_pub_data["pkg_name"].asString();
                params = app_pub_data["params"].asString();
                rlt = mrParamYaml.setPkgParams(pkgName,params)?ERRORCODE_SUCCESS:ERRORCODE_FAIL;
                sendflag = true;
            }
        }
        else if(0 == app_pub_name.compare("pkg_params_restore"))
        {            
            if(!app_pub_data.isNull() && app_pub_data["pkg_name"].isString())
            {
                pkgName = app_pub_data["pkg_name"].asString();
                rlt = mrParamYaml.restorePkgParams(pkgName)?ERRORCODE_SUCCESS:ERRORCODE_FAIL;
                sendflag = true;
            }
        }
        if(sendflag)
        {
            sendMsg(rlt,app_pub_name,sendData);
        }
     }
}

void MrParamNode::sendMsg(int rlt, std::string cmdName, Json::Value data)
{
     std_msgs::String send_msg;
     if(rlt == ERRORCODE_SUCCESS)
     {
         send_msg = writeJsonStr(cmdName,ERRORCODE_SUCCESS,"success",data);
     }
     else
     {
         send_msg = writeJsonStr(cmdName,ERRORCODE_FAIL,"arguments invalid",data);
     }
     mr_param_pub.publish(send_msg);
}

bool MrParamNode::readJsonStr(const std_msgs::String::ConstPtr& msg)
{
   Json::Reader reader;
   Json::Value value;
   bool rlt = false;
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
           rlt = true; 
       }
   }
   return rlt;
}

std_msgs::String MrParamNode::writeJsonStr(std::string name, int errorcode, std::string msg, Json::Value data)
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


int main(int argc, char **argv)
{ 
   ros::init(argc, argv, "mr_param_node");
   MrParamNode mrParamNode;
   ros::spin();
   return 0;
}