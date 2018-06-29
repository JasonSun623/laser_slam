#include "robot_state_keeper/robot_state_keeper.h"



RobotStateKeeper::RobotStateKeeper()
{
    //robot_state_keeper node
    ros::NodeHandle n("~");
	ros::param::set("system_version",VERSION_INFO);
    //topic state_pub and cmd_pub should be system root namespace.
    state_pub = n.advertise<std_msgs::String>("/app_sub", 10);
    cmd_sub = n.subscribe("/app_pub", 1, &RobotStateKeeper::cmd_callback,this);

    //register service to support other modules register.
    register_srv = n.advertiseService("register_robot_modules",&RobotStateKeeper::register_robot_modules,this);
	

    //topic connection_check publish robot beacon.
    n.param("beacon_freq",beacon_freq,0.2);
    if(beacon_freq <=0)
    {
        beacon_freq = 1.0;
    }
    n.param("robot_id",robot_id,std::string("0"));
    beacon_pub = n.advertise<std_msgs::Header>("/connection_check",10);

    //timer to send beacon
    beacon_timer = n.createTimer(ros::Duration(1/beacon_freq),&RobotStateKeeper::timer_callback,this);
}

//timer to send beacon
void RobotStateKeeper::timer_callback(const ros::TimerEvent &e)
{
    static u_int32_t count = 0;
    std_msgs::Header h;
    h.stamp = ros::Time::now();
    h.seq = count++;
    h.frame_id = "robot_"+robot_id;
    beacon_pub.publish(h);
    ROS_DEBUG("robot state keeper connection check publish");
}

//handle json data from cmd_pub topic.
void RobotStateKeeper::cmd_handle(Json::Value &value)
{
    Json::Value msg;
    std_msgs::String state_msg;
    Json::StyledWriter fast;
    bool feedback = false;

    if(!value[MSG_PUB_NAME].isString())
    {
        return;
    }

    if(value[MSG_PUB_NAME].asString() == CMD_ROBOT_MODULES_GET)
    {
        ROS_DEBUG("request get robot modules");

        int i = 0;
        Json::Value m;
        Json::Value module;
        for(i = 0;i < modules.size();i++)
        {
            module["name"]=modules[i].name;
            module["error_code"]=modules[i].error_code;
            module["enabled"]=modules[i].enabled;
            module["info"]=modules[i].info;
            m.append(module);
        }
        msg[MSG_ERROR_CODE] = Json::Value(0);
        msg[MSG_MSG];
        msg[MSG_DATA] = m;
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString() == CMD_ROBOT_VERSION_INFO_GET)
    {
        ROS_DEBUG("request get robot version info");
        msg[MSG_ERROR_CODE] = Json::Value(0);
        msg[MSG_MSG];
        msg[MSG_DATA] = VERSION_INFO;
        feedback = true;
    }

    if(feedback)
    {
        msg[MSG_SUB_NAME] = value[MSG_PUB_NAME];
        state_msg.data = fast.write(msg);
        state_pub.publish(state_msg);
    }
}

//callback to handle cmd_pub topic command json data
void RobotStateKeeper::cmd_callback(const std_msgs::String::ConstPtr& msg)
{
    std::string test =msg->data;
    Json::Reader reader;
    Json::Value value;
    if(reader.parse(test,value))
    {
       if(value.isNull())
       {
           ROS_DEBUG("pub_name value is empty");
       }
       else
       {
           cmd_handle(value);
       }
    }

}

//for other modules register
bool RobotStateKeeper::register_robot_modules(robot_state_keeper::RegisterState::Request &req,
                                        robot_state_keeper::RegisterState::Response &res)
{
    int i = 0;
    //ROS_DEBUG("service register a module:%s,size:%lu",req.state.name.c_str(),modules.size());
    for(i = 0;i < modules.size();i++)
    {
        if(modules[i].name == req.state.name)
        {
            modules[i] = req.state;
            res.error_code = 0;
            return true;
        }

    }
    if(i == modules.size())
    {
        modules.push_back(req.state);
        res.error_code = 0;
        return true;
    }
    return false;
}

