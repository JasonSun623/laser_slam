#include "motion_planner/motion_planner.h"


/*dynamic_reconfigure::ReconfigureRequest srv_req;
dynamic_reconfigure::ReconfigureResponse srv_resp;
dynamic_reconfigure::DoubleParameter double_param;
dynamic_reconfigure::Config conf;
//inflation_radius
double_param.name = "inflation_radius";
double_param.value = 0.6;
conf.doubles.push_back(double_param);

srv_req.config = conf;

ros::service::call("/move_base/global_costmap/inflation_layer/set_parameters", srv_req, srv_resp);
*/


template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

MotionPlanner::MotionPlanner()
{
    ROS_INFO("initialize motion planner");
    /********motion_planner node******/
    ros::NodeHandle n("~");

    //publish robot state feedback
    state_pub = n.advertise<std_msgs::String>("/app_sub", 10);

    //subscribe app command
    cmd_sub = n.subscribe("/app_pub", 20, &MotionPlanner::cmd_callback,this);

    //subscribe current map list
    map_list_sub = n.subscribe("/map_list_pub", 5, &MotionPlanner::map_list_callback,this);

    //service of register motion planners.
    planners_srv = n.advertiseService("planners_register",&MotionPlanner::planners_service,this);

    mark_guide_client = n.serviceClient<motion_planner::MotionOp>("/marker_guide_op");

    open_motion_client = n.serviceClient<motion_planner::MotionOp>("/open_motion_op");

    lane_following_motion_client = n.serviceClient<motion_planner::MotionOp>("/lane_follower_node/missions");

    fixpath_nav_client = n.serviceClient<motion_planner::MotionOp>("/fixpath_nav_op");

    param_recovery_client = n.serviceClient<motion_planner::MotionOp>("/map_server_mrobot/region_params_changer/param_update");

    motion_status_sub = n.subscribe("/motion_status",5,&MotionPlanner::motion_status_callback,this);


    /*******move_base node*******/
    ros::NodeHandle move_base_nh("move_base");

    //publish goal to start navigation
    action_goal_pub = move_base_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    //service to pause or resume motion planner.
    pause_client = move_base_nh.serviceClient<move_base::Pause>("motion_control");

    //service to configure move base params.
    configure_move_base_client = move_base_nh.serviceClient<move_base::ConfigParam>("configure_param");

    //service to pause or resume costmap update
    costmap_pause_update_client = move_base_nh.serviceClient<move_base::Pause>("costmap_pause_update");

    //publish to cancel navigation.
    cancel_pub =  move_base_nh.advertise<actionlib_msgs::GoalID>("cancel",1);

    movebase_status_sub = move_base_nh.subscribe("status",100,&MotionPlanner::movebase_status_callback,this);

    /********robot_state_keeper node******/
    ros::NodeHandle robot_nh("robot_state_keeper");

    //service to register this module.
    robot_register_client = robot_nh.serviceClient<robot_state_keeper::RegisterState>("register_robot_modules");

    //get configure file path.
    n.param("config_file_path",config_file_path,std::string("/home/robot/catkin_ws/src/motion_planner/configs/config.yaml"));

    //publish nav state
    nav_status_pub =  move_base_nh.advertise<std_msgs::UInt8MultiArray>("nav_status",1);

    //read config file to initial parameters.
    bool success = init_config_params();

    //initial this module state.
    if(success)
    {
        motion_state.enabled = 0;
    }
    else
    {
        motion_state.enabled = 1;
    }
    motion_state.name = "motion_planner";
    motion_state.error_code = 0;
    motion_state.info = "robot navigation control center";
    costmap_update_pause_flag = false;
    nav_status = NAV_READY;
    current_nav_task = MOVE_IDLE;
    n.setParam("nav_status",nav_status);

    //pub nav status
    nav_status_thread = new boost::thread(boost::bind(&MotionPlanner::navStatusPubThread, this));

    //start a new thread to resigter this module to robot state keeper node
    register_thread = new boost::thread(boost::bind(&MotionPlanner::registerThread, this));

    ROS_INFO("motion planner initialize finished");
}


void MotionPlanner::pub_nav_state(u_int8_t state)
{
	std_msgs::UInt8MultiArray nav_state_msg;

	nav_state_msg.data.push_back(state);
	nav_status_pub.publish(nav_state_msg);
}

void MotionPlanner::navStatusPubThread()
{
	ros::Rate nav_status_pub_rate(NAV_STATUS_PUB_RTATE);
	while(ros::ok())
	{
		pub_nav_state(nav_status);
		nav_status_pub_rate.sleep();
	}
}

//initial parameters through reading config file.
bool MotionPlanner::init_config_params()
{
    std::ifstream fin(config_file_path.c_str());
    if(!fin)
    {
        return false;
    }

    //load yaml file.
    config_info = YAML::Load(fin);
    if(config_info)
    {
        if(config_info[PARAM_MAX_VX] && config_info[PARAM_MAX_VY] &&
                config_info[PARAM_MAX_VTH] && config_info[PARAM_DEFAULT_PLANNER])
        {
            config_info[PARAM_MAX_VX] >> max_x_vel;
            config_info[PARAM_MAX_VY] >> max_y_vel;
            config_info[PARAM_MAX_VTH] >> max_th_vel;
            config_info[PARAM_DEFAULT_PLANNER] >> default_planner;
            return true;
        }
    }
    return false;
}

//thread to register this module
void MotionPlanner::registerThread()
{
    ros::Rate r(REGISTER_RATE);
    bool registered = false;
    while(ros::ok())
    {
        //call service
        robot_state_keeper::RegisterState srv;
        srv.request.state = motion_state;
        if(!registered)
        {
            if(robot_register_client.call(srv))
            {
                ROS_DEBUG("motion planner register ok!");
                registered = true;
            }
            else
            {
                ROS_DEBUG("motion planner register failed!");
            }
        }
        r.sleep();
    }
}

//call move_base node service to pause or resume motion.
bool MotionPlanner::motion_pause(bool pause)
{
    if(default_planner == MOVE_BASE_PLANNER)
    {
        move_base::Pause srv;
        srv.request.pause = pause;
        if(pause_client.call(srv))
        {
            ROS_DEBUG("motion pause or resume operation ok!");
            return true;
        }
        else
        {
            ROS_DEBUG("motion pause or resume operation failed!");
        }
    }
    ROS_DEBUG("default planner:[%s] is error",default_planner.c_str());
    return false;
}

//call move_base node service to pause or resume costmap update.
bool MotionPlanner::costmap_pause_update(bool pause)
{
    if(default_planner == MOVE_BASE_PLANNER)
    {
        move_base::Pause srv;
        srv.request.pause = pause;
        if(costmap_pause_update_client.call(srv))
        {
            ROS_DEBUG("costmap pause or resume operation ok!");
            return true;
        }
        else
        {
            ROS_DEBUG("costmap pause or resume operation failed!");
        }
    }
    ROS_DEBUG("default planner:[%s] is error",default_planner.c_str());
    return false;
}

//set config value and update yaml config file.
Json::Value MotionPlanner::set_config_param(const Json::Value &value)
{
    Json::Value msg;  //json msg for feedback
    Json::Value data; //json msg'data for feedback

    msg[MSG_ERROR_CODE] = Json::Value(0);
    msg[MSG_MSG];

    //json check value is not null
    if(value.isNull())
    {
        msg[MSG_ERROR_CODE] = Json::Value(1);
        msg[MSG_MSG] = "json data have no valid info";
        msg[MSG_DATA];
        return msg;
    }

    //json check value param_name and value exist and is string type
    if(!(value[PARAM_PARAM_NAME].isString() && value[MSG_VALUE].isString()))
    {
        msg[MSG_ERROR_CODE] = Json::Value(1);
        msg[MSG_MSG] = "json data have wrong info";
        msg[MSG_DATA];
        return msg;
    }

    data[PARAM_PARAM_NAME] = value[PARAM_PARAM_NAME];

    std::ofstream fout(config_file_path.c_str());
    if(!fout)
    {
        msg[MSG_ERROR_CODE] = Json::Value(2);
        msg[MSG_MSG] = "cannot open config file";
        msg[MSG_DATA];
        return msg;
    }

    std::string value_string = value[MSG_VALUE].asString();
    if(!value_string.compare(""))
    {
        msg[MSG_ERROR_CODE] = Json::Value(3);
        msg[MSG_MSG] = "json param wrong";
        msg[MSG_DATA];
        fout.close();
        return msg;
    }

    if(value[PARAM_PARAM_NAME].asString() == PARAM_DEFAULT_PLANNER)
    {
        //if set default_planner param,
        default_planner = value_string;
        data[MSG_VALUE] = default_planner;
        //update yaml config file
        config_info[PARAM_DEFAULT_PLANNER] = default_planner;
        fout << config_info;
    }
    else if(value[PARAM_PARAM_NAME].asString() == PARAM_MAX_VX)
    {
        //if set max_x_vel param,
        max_x_vel = atof(value_string.c_str());
        data[MSG_VALUE] = max_x_vel;

        //update yaml config file
        config_info[PARAM_MAX_VX] = max_x_vel;
        fout << config_info;
    }
    else if(value[PARAM_PARAM_NAME].asString() == PARAM_MAX_VY)
    {
        //if set max_y_vel param,
        max_y_vel = atof(value_string.c_str());
        data[MSG_VALUE] = max_y_vel;

        //update yaml config file
        config_info[PARAM_MAX_VY] = max_y_vel;
        fout << config_info;
    }
    else if(value[PARAM_PARAM_NAME].asString() == PARAM_MAX_VTH)
    {
        //if set max_th_vel param,
        max_th_vel = atof(value_string.c_str());
        data[MSG_VALUE] = max_th_vel;

        //update yaml config file
        config_info[PARAM_MAX_VTH] = max_th_vel;
        fout << config_info;
    }
    else
    {
        data[MSG_VALUE];
        msg[MSG_ERROR_CODE] = Json::Value(1);
        msg[MSG_MSG] = "Don't exist param";
    }
    msg[MSG_DATA]=data;
    fout.close();

    return msg;
}

//get config parameter
Json::Value MotionPlanner::get_config_param(const Json::Value &value)
{
    Json::Value msg;
    Json::Value data;

    msg[MSG_ERROR_CODE] = Json::Value(0);
    msg[MSG_MSG];

    if(value.isNull())
    {
        msg[MSG_ERROR_CODE] = Json::Value(1);
        msg[MSG_MSG] = "json value is null";
        msg[MSG_DATA];
        return msg;
    }
    if(!value[PARAM_PARAM_NAME].isString())
    {
        msg[MSG_ERROR_CODE] = Json::Value(2);
        msg[MSG_MSG] = "json value is wrong";
        msg[MSG_DATA];
        return msg;
    }
    data[PARAM_PARAM_NAME] = value[PARAM_PARAM_NAME];

    if(value[PARAM_PARAM_NAME].asString() == PARAM_DEFAULT_PLANNER)
    {
        data[MSG_VALUE] = default_planner;
    }
    else if(value[PARAM_PARAM_NAME].asString() == PARAM_MAX_VX)
    {
        data[MSG_VALUE] = max_x_vel;
    }
    else if(value[PARAM_PARAM_NAME].asString() == PARAM_MAX_VY)
    {
        data[MSG_VALUE] = max_y_vel;
    }
    else if(value[PARAM_PARAM_NAME].asString() == PARAM_MAX_VTH)
    {
        data[MSG_VALUE] = max_th_vel;
    }
    else
    {
        data[MSG_VALUE];
        msg[MSG_ERROR_CODE] = Json::Value(3);
        msg[MSG_MSG]="Don't exist param";
    }
    msg[MSG_DATA]=data;
    return msg;
}

bool MotionPlanner::change_nav_status(nav_status_e status)
{
	if(status == NAV_READY)
	{
		// planner recover will remove all cmds if status change to nav_ready
		this->planner_recover.ClearMission();
	}
    if(nav_status != status)
    {
        Json::Value msg;
        std_msgs::String state_msg;
        Json::StyledWriter fast;
        msg[MSG_SUB_NAME] = FB_MOTION_PLANNER_EVENT;
        msg[MSG_MSG];
        Json::Value data;
        data["event_type"] = Json::Value(1);
        data["status_from"] = nav_status;
        data["status_to"] = status;
        msg[MSG_DATA] = data;
        msg[MSG_ERROR_CODE] = Json::Value(0);
        //report event;
        state_msg.data = fast.write(msg);
        state_pub.publish(state_msg);
        nav_status = status;
        ros::NodeHandle n("~");
        n.setParam("nav_status",nav_status);
    }
    return true;
}

//handle received json command
void MotionPlanner::cmd_handle(const Json::Value &value)
{
    Json::Value msg;
    std_msgs::String state_msg;
    Json::StyledWriter fast;
    bool feedback = false;
    bool flag = false;

    //json: if value item is not string,when us asString function,the program will crash.
    if(!value[MSG_PUB_NAME].isString())
    {
        return;
    }

    if(value[MSG_PUB_NAME].asString()==CMD_NAV_GOAL_PAUSE)
    {
        ROS_INFO("request nav goal pause");
        if(MOVE_BASE_PLANNER == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                bool success = motion_pause(true);
                if(success)
                {
                    change_nav_status(NAV_PAUSE);
                    msg[MSG_ERROR_CODE]= Json::Value(0);
                    msg[MSG_MSG];
                }
                else
                {
                    msg[MSG_ERROR_CODE]= Json::Value(1);
                    msg[MSG_MSG] = "set nav goal pause failed";
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set nav goal pause failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not move base planner";
        }
        msg[MSG_DATA];
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_NAV_GOAL_RESUME)
    {
        ROS_INFO("request nav goal resume");

        if(MOVE_BASE_PLANNER == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                bool success = motion_pause(false);
                msg[MSG_DATA];
                if(success)
                {
                    change_nav_status(NAV_MOVING);
                    msg[MSG_ERROR_CODE]= Json::Value(0);
                    msg[MSG_MSG];
                }
                else
                {
                    msg[MSG_ERROR_CODE]= Json::Value(1);
                    msg[MSG_MSG] = "set nav goal resume failed";
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set nav goal resume failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not move base planner";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_NAV_COSTMAP_PAUSE)
    {
        ROS_INFO("request nav costmap pause");

        bool success = costmap_pause_update(true);
        msg[MSG_DATA];
        if(success)
        {
            costmap_update_pause_flag = true;
            msg[MSG_ERROR_CODE]= Json::Value(0);
            msg[MSG_MSG];
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(1);
            msg[MSG_MSG] = "set nav costmap pause failed";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_NAV_COSTMAP_RESUME)
    {
        ROS_INFO("request nav costmap resume");

        bool success = costmap_pause_update(false);
        msg[MSG_DATA];
        if(success)
        {
            costmap_update_pause_flag = false;
            msg[MSG_ERROR_CODE]= Json::Value(0);
            msg[MSG_MSG];
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(1);
            msg[MSG_MSG] = "set nav costmap resume failed";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_GET_COSTMAP_PAUSE_STATUS)
    {
        ROS_INFO("get costmap update pause status");
        msg[MSG_DATA]=costmap_update_pause_flag;
        msg[MSG_ERROR_CODE]= Json::Value(0);
        msg[MSG_MSG];
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_SET_PARAM)
    {
        ROS_INFO("request set motion planner param");

        Json::Value data;
        data = value[MSG_DATA];
        msg = set_config_param(data);
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_GET_PARAM)
    {
        ROS_INFO("request get motion planner param");

        Json::Value data;
        data = value[MSG_DATA];
        msg = get_config_param(data);
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_NAV_GOAL_START)
    {
        ROS_INFO("request nav to a goal");

        Json::Value data;
        data = value[MSG_DATA];
        msg[MSG_DATA] = value[MSG_DATA];
        if(MOVE_IDLE == current_nav_task || MOVE_BASE_PLANNER == current_nav_task )
        {
            if(data["map"].isString() &&(data["th"].isDouble() || data["th"].isInt()) &&
                    (data["x"].isDouble() || data["x"].isInt()) &&
                    (data["y"].isDouble() || data["y"].isInt()) )
            {
                if(default_planner == MOVE_BASE_PLANNER)
                {
                    if((map_list_info.current_map == data["map"].asString()))
                    {
                        move_base_msgs::MoveBaseActionGoal action_goal;
                        action_goal.header.stamp = ros::Time::now();
                        geometry_msgs::PoseStamped goal;
                        goal.header.frame_id = "map";
                        goal.header.stamp = ros::Time::now();
                        double yaw = 0.0;
                        double x = 0.0;
                        double y = 0.0;
                        if(data["th"].isDouble())
                        {
                            yaw = data["th"].asDouble();
                        }
                        else
                        {
                            yaw = data["th"].asInt();
                        }
                        if(data["x"].isDouble())
                        {
                            x = data["x"].asDouble();
                        }
                        else
                        {
                            x = data["x"].asInt();
                        }
                        if(data["y"].isDouble())
                        {
                            y = data["y"].asDouble();
                        }
                        else
                        {
                            y = data["y"].asInt();
                        }
                        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                        goal.pose.position.x = x;
                        goal.pose.position.y = y;
                        action_goal.goal.target_pose = goal;

                        action_goal_pub.publish(action_goal);
                        if(NAV_PAUSE != nav_status)
                        {
                            change_nav_status(NAV_MOVING);
                        }
                        msg[MSG_ERROR_CODE]= Json::Value(0);
                        msg[MSG_MSG];
                        latest_goal.x = x;
                        latest_goal.y = y;
                        latest_goal.theta = yaw;
                        latest_goal_map = map_list_info.current_map;
                        latest_goal_id = data["name"].asString();
                        current_nav_task = MOVE_BASE_PLANNER;
                    }
                    else
                    {
                        msg[MSG_ERROR_CODE]= Json::Value(2);
                        msg[MSG_MSG] = "goal's map param is wrong,current:"+map_list_info.current_map+"param:"+data["map"].asString();
                    }
                }
                else
                {
                    msg[MSG_ERROR_CODE]= Json::Value(1);
                    msg[MSG_MSG] = "don't recognize the using motion planner,default planner:"+default_planner;
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(3);
                msg[MSG_MSG] = "json data is wrong";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not idle";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_NAV_GOAL_CANCEL)
    {
        ROS_INFO("request cancel nav");

        if(MOVE_BASE_PLANNER == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                if(default_planner == MOVE_BASE_PLANNER)
                {
                    bool success = motion_pause(false);
                    if(success)
                    {
                        actionlib_msgs::GoalID first_goal;
                        cancel_pub.publish(first_goal);
                        change_nav_status(NAV_READY);
                        msg[MSG_ERROR_CODE]= Json::Value(0);
                        msg[MSG_MSG];
                        current_nav_task = MOVE_IDLE;
                    }
                    else
                    {
                        msg[MSG_ERROR_CODE]= Json::Value(2);
                        msg[MSG_MSG]="cancel navigation:motion pause resume failed!";
                    }
                }
                else
                {
                    msg[MSG_ERROR_CODE]= Json::Value(1);
                    msg[MSG_MSG] = "don't recognize the using motion planner";
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(3);
                msg[MSG_MSG] = "cancel cmd failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not move base planner";
        }
        msg[MSG_DATA];

        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_GET_MOTION_PLANNERS)
    {
        ROS_INFO("request get motion planners");

        int i = 0;
        Json::Value m;
        Json::Value planner;
        for(i = 0;i < planners.size();i++)
        {
            planner["name"]=planners[i].name;
            planner["error_code"]=planners[i].error_code;
            planner["enabled"]=planners[i].enabled;
            planner["info"]=planners[i].info;
            m.append(planner);
        }
        msg[MSG_ERROR_CODE]=Json::Value(0);
        msg[MSG_MSG];
        msg[MSG_DATA] = m;
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_GET_MOTION_STATUS)
    {
        ROS_INFO("request motion status");

        switch(last_motion_status)
        {
        case SUCCEEDED:
            msg[MSG_MSG] = last_goal_id + ":" + "goal reached";
            break;
        case ACTIVE:
            msg[MSG_MSG] = last_goal_id + ":" + "moving";
            break;
        default:
            msg[MSG_MSG] = "default moving";
            break;
        }
        Json::Value data;
        data["x"] = latest_goal.x;
        data["y"] = latest_goal.y;
        data["th"] = latest_goal.theta;
        data["map"] = latest_goal_map;
        data["name"] = latest_goal_id;
        msg[MSG_DATA] = data;
        msg[MSG_ERROR_CODE]= Json::Value(last_motion_status);

        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString() == CMD_OP_MOVE_BASE_PARAM)
    {
        ROS_INFO("request op move_base param");

        if(default_planner == MOVE_BASE_PLANNER)
        {
            Json::Value data;
            Json::StyledWriter fast;
            data = value[MSG_DATA];
            move_base::ConfigParam srv;
            srv.request.cmd = fast.write(data);

            if(configure_move_base_client.call(srv))
            {
                ROS_DEBUG("request op move_base param ok!");
                Json::Reader reader;
                Json::Value value;
                msg[MSG_ERROR_CODE]= Json::Value(0);
                msg[MSG_MSG];
                reader.parse(srv.response.feedback,value);
                msg[MSG_DATA] = value;
            }
            else
            {
                ROS_DEBUG("request op move_base param failed!");
                msg[MSG_ERROR_CODE]= Json::Value(1);
                msg[MSG_MSG] = "request op move_base param failed";
            }
        }
        else
        {
            ROS_DEBUG("request op move_base param failed!");
            msg[MSG_ERROR_CODE]= Json::Value(2);
            msg[MSG_MSG] = "current planner is not right";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_MARKER_GUIDE_PAUSE)
    {
        ROS_INFO("request mark guide pause");

        if(MOVE_MARKER_GUIDE == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = mark_guide_op(value,msg);
                if(flag&&(NAV_MOVING == nav_status))
                {
                	std::string cmd = (value.toStyledString());
                	this->planner_recover.CacheMission(cmd);
                }
                change_nav_status(NAV_PAUSE);
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set mark guide pause failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not right";
        }
        msg[MSG_DATA];
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_MARKER_GUIDE_RESUME)
    {
        ROS_INFO("request marker guide resume");

        if(MOVE_MARKER_GUIDE == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = mark_guide_op(value,msg);
                if(flag&&(NAV_PAUSE==nav_status))
                {
                	this->planner_recover.PopTop();
                }
                change_nav_status(NAV_MOVING);

            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set marker guide resume failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not move base planner";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_MARKER_GUIDE_START)
    {
        ROS_INFO("request marker guide start");
        if(MOVE_IDLE == current_nav_task)
        {
            //set marker guide start
            flag = mark_guide_op(value,msg);
            if(flag)
            {
            	ROS_INFO("Marker guide start succeed");
            	this->planner_recover.ClearMission();
                std::string cmd = (value.toStyledString());
            	this->planner_recover.CacheMission(cmd);

                current_nav_task = MOVE_MARKER_GUIDE;
                change_nav_status(NAV_MOVING);
            }
            else
            {   
                ROS_INFO("Marker guide start failed");
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not idle";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_MARKER_GUIDE_CANCEL)
    {
        ROS_INFO("request marker guide cancel");
        if(MOVE_MARKER_GUIDE == current_nav_task)
        {
            //set marker guide cancel
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
		        flag = mark_guide_op(value,msg);
		        if(flag)
                {
                    current_nav_task = MOVE_IDLE;

                    change_nav_status(NAV_READY);
		            ROS_INFO("Marker guide cancel succeed");
		        }
		        else
		        {
		            ROS_INFO("Marker guide cancel failed");
		        }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set marker guide cancel failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not marker guide";
        }
        feedback = true;
    }

    //fixpath motion op
    else if(value[MSG_PUB_NAME].asString()==CMD_FIXPATH_MOTION_PAUSE)
    {
        ROS_INFO("request fixpath motion pause");

        if(MOVE_FIXPATH_MOTION == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = fixpath_motion_op(value,msg);
                if(flag)
                {
                	if(NAV_MOVING == nav_status)
                	{
				// planner recover will store the cmd of pause,if last cmd is 
				// nav_moving
                		std::string cmd = (value.toStyledString());
                		this->planner_recover.CacheMission(cmd);
                	}
                    change_nav_status(NAV_PAUSE);
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set fixpath motion pause failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not right";
        }
        msg[MSG_DATA];
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_FIXPATH_MOTION_RESUME)
    {
        ROS_INFO("request fixpath motion resume");

        if(MOVE_FIXPATH_MOTION == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = fixpath_motion_op(value,msg);
                if(flag)
                {
                	if(NAV_PAUSE == nav_status)
                	{
				// planner recover will remove last cmd ,if last cmd is 
				// nav_pause
                		this->planner_recover.PopTop();
                	}
                    change_nav_status(NAV_MOVING);
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set fixpath motion resume failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not fixpath planner";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString() == CMD_FIXPATH_MOTION_START)
    {
        ROS_INFO("request fix motion start");
        if((MOVE_IDLE == current_nav_task) || (MOVE_FIXPATH_MOTION == current_nav_task))
        {
            //set fixpath motion start
            flag = fixpath_motion_op(value,msg);
            if(flag)
            {
			// planner recover will remove all cmds and store the new cmd for staring
        		this->planner_recover.ClearMission();
        		std::string cmd = (value.toStyledString());
                //ROS_ERROR("fixpath motion start succeed %s",cmd.c_str());
        		this->planner_recover.CacheMission(cmd);
                current_nav_task = MOVE_FIXPATH_MOTION;
                change_nav_status(NAV_MOVING);
            }
            else
            {
                ROS_INFO("fixpath motion start failed");
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not idle";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_FIXPATH_MOTION_CANCEL)
    {
        ROS_INFO("request fixpath motion cancel");
        if(MOVE_FIXPATH_MOTION == current_nav_task)
        {
            //set open motion cancel
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = fixpath_motion_op(value,msg);
                if(flag)
                {
                    current_nav_task = MOVE_IDLE;

                    change_nav_status(NAV_READY);
                    ROS_INFO("fixpath motion cancel succeed");
                }
                else
                {
                    ROS_INFO("fixpath motion cancel failed");
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set fixpath motion cancel failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not fixpath motion";
        }
        feedback = true;
    }

    //open motion op
    else if(value[MSG_PUB_NAME].asString()==CMD_OPEN_MOTION_PAUSE)
    {
        ROS_INFO("request open motion pause");

        if(MOVE_OPEN_MOTION == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = open_motion_op(value,msg);
                if(flag)
                {
                	if(NAV_MOVING == nav_status)
                	{
                		std::string cmd = (value.toStyledString());
                		this->planner_recover.CacheMission(cmd);
                	}
                	change_nav_status(NAV_PAUSE);
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set open motion pause failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not right";
        }
        msg[MSG_DATA];
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_OPEN_MOTION_RESUME)
    {
        ROS_INFO("request open motion resume");

        if(MOVE_OPEN_MOTION == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = open_motion_op(value,msg);
                if(flag)
                {
                	if(NAV_PAUSE == nav_status)
                		this->planner_recover.PopTop();
                    change_nav_status(NAV_MOVING);
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set open motion resume failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not move base planner";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_OPEN_MOTION_START)
    {
        ROS_INFO("request open motion start");
        if(MOVE_IDLE == current_nav_task)
        {
            //set open motion start
            flag = open_motion_op(value,msg);
            if(flag)
            {   ROS_INFO("Open motion start succeed");
    			this->planner_recover.ClearMission();
    			std::string cmd = (value.toStyledString());
                ROS_ERROR("misson rev %s",cmd.c_str());
    			this->planner_recover.CacheMission(cmd);
                current_nav_task = MOVE_OPEN_MOTION;
                change_nav_status(NAV_MOVING);
            }
            else
            {
                ROS_INFO("Open motion start failed");
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not idle";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_OPEN_MOTION_CANCEL)
    {
        ROS_INFO("request open motion cancel");
        if(MOVE_OPEN_MOTION == current_nav_task)
        {
            //set open motion cancel
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = open_motion_op(value,msg);
                if(flag)
                {
                    current_nav_task = MOVE_IDLE;
                    change_nav_status(NAV_READY);
                    ROS_INFO("open motion cancel succeed");
                }
                else
                {
                    ROS_INFO("open motion cancel failed");
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set open motion cancel failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not open motion";
        }
        feedback = true;
    }
    // lane follower motion
    else if(value[MSG_PUB_NAME].asString()==CMD_LANE_FOLLOWING_PAUSE)
    {
        ROS_INFO("request lane following motion pause");

        if(MOVE_LANE_FOLLOWING_MOTION == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = lane_following_motion_op(value,msg);
                if(flag)
                {
                	if(NAV_MOVING == nav_status)
                	{
                        std::string cmd = (value.toStyledString());
                    	this->planner_recover.CacheMission(cmd);
                	}
                    change_nav_status(NAV_PAUSE);
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set lane following motion pause failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not right";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_LANE_FOLLOWING_RESUME)
    {
        ROS_INFO("request lane following motion resume");

        if(MOVE_LANE_FOLLOWING_MOTION == current_nav_task)
        {
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = lane_following_motion_op(value,msg);
                if(flag)
                {
                	if(NAV_PAUSE == nav_status)
                	{
                    	this->planner_recover.PopTop();
                	}
                    change_nav_status(NAV_MOVING);
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set lane following motion resume failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not lane following planner";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_LANE_FOLLOWING_FORWARD)
    {
        ROS_INFO("request lane following motion start");
        if(MOVE_IDLE == current_nav_task)
        {
            //set open motion start
            flag = lane_following_motion_op(value,msg);
            if(flag)
            {
            	ROS_INFO("Lane following motion start succeed");
            	this->planner_recover.ClearMission();
            	std::string cmd = (value.toStyledString());
            	this->planner_recover.CacheMission(cmd);
                current_nav_task = MOVE_LANE_FOLLOWING_MOTION;
                change_nav_status(NAV_MOVING);
            }
            else
            {
                ROS_INFO("Lane following motion start failed");
            }
        }
        else
        {
			
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not idle";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_LANE_FOLLOWING_BACKWARD)
    {
        ROS_INFO("request lane following motion start");
        if(MOVE_IDLE == current_nav_task)
        {
            //set open motion start
            flag = lane_following_motion_op(value,msg);
            if(flag)
            {   ROS_INFO("Lane following motion start succeed");
        		this->planner_recover.ClearMission();
        		std::string cmd = (value.toStyledString());
        		this->planner_recover.CacheMission(cmd);
                current_nav_task = MOVE_LANE_FOLLOWING_MOTION;
                change_nav_status(NAV_MOVING);
            }
            else
            {
                ROS_INFO("Lane following motion start failed");
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not idle";
        }
        feedback = true;
    }
    else if(value[MSG_PUB_NAME].asString()==CMD_LANE_FOLLOWING_CANCEL)
    {
        ROS_INFO("request lane following motion cancel");
        if(MOVE_LANE_FOLLOWING_MOTION == current_nav_task)
        {
            //set open motion cancel
            if((NAV_PAUSE==nav_status) || (NAV_MOVING == nav_status))
            {
                flag = lane_following_motion_op(value,msg);
                if(flag)
                {

                    current_nav_task = MOVE_IDLE;
                    change_nav_status(NAV_READY);
                    ROS_INFO("lane following motion cancel succeed");
                }
                else
                {
                    ROS_INFO("lane following motion cancel failed");
                }
            }
            else
            {
                msg[MSG_ERROR_CODE]= Json::Value(2);
                msg[MSG_MSG] = "set lane following motion cancel failed,nav status not match";
            }
        }
        else
        {
            msg[MSG_ERROR_CODE]= Json::Value(10);
            msg[MSG_MSG] = "current nav task is not lane following motion";
        }
        feedback = true;
    }
    if(feedback)
    {
        //if handle a valid command of this module,publish feedback.
        msg[MSG_SUB_NAME] = value[MSG_PUB_NAME];
        state_msg.data = fast.write(msg);
        state_pub.publish(state_msg);
    }
}

bool MotionPlanner::mark_guide_op(Json::Value value, Json::Value &msg)
{
    Json::StyledWriter fast;
    motion_planner::MotionOp srv;

    srv.request.cmd = fast.write(value);

    if(mark_guide_client.call(srv))
    {
        ROS_DEBUG("motion pause or resume operation ok!");
        Json::Reader reader;
        reader.parse(srv.response.result,msg);
        if(0 == msg[MSG_ERROR_CODE].asInt())
        {

            return true;
        }
        else
        {
            ROS_INFO("operation marker guide failed");
        }
    }
    else
    {
        msg[MSG_ERROR_CODE]= Json::Value(100);
        msg[MSG_MSG] = "call motion pause or resume operation failed!";
        ROS_DEBUG("motion pause or resume operation failed!");
    }

    return false;
}
bool MotionPlanner::lane_following_motion_op(const Json::Value &value, Json::Value &msg)
{
    Json::StyledWriter fast;
    motion_planner::MotionOp srv;

    srv.request.cmd = fast.write(value);
    if(lane_following_motion_client.call(srv))
    {
        ROS_INFO("lane following motion op ok!");
        Json::Reader reader;
        ROS_INFO("lane following motion op:%s",srv.response.result.c_str());
        reader.parse(srv.response.result,msg);
        if(0 == msg[MSG_ERROR_CODE].asInt())
        {
            return true;
        }
        else
        {
            ROS_INFO("operation lane following motion failed");
        }
    }
    else
    {
        msg[MSG_ERROR_CODE]= Json::Value(100);
        msg[MSG_MSG] = "call lane following motion operation failed!";
        ROS_INFO("lane following motion operation failed!");
    }

    return false;
}
bool MotionPlanner::open_motion_op(Json::Value value, Json::Value &msg)
{
    Json::StyledWriter fast;
    motion_planner::MotionOp srv;

    srv.request.cmd = fast.write(value);
    if(open_motion_client.call(srv))
    {
        ROS_INFO("open motion op ok!");
        Json::Reader reader;
        ROS_INFO("open motion op:%s",srv.response.result.c_str());
        reader.parse(srv.response.result,msg);
        if(0 == msg[MSG_ERROR_CODE].asInt())
        {
            return true;
        }
        else
        {
            ROS_INFO("operation open motion failed");
        }
    }
    else
    {
        msg[MSG_ERROR_CODE]= Json::Value(100);
        msg[MSG_MSG] = "call open motion operation failed!";
        ROS_INFO("open open motion operation failed!");
    }

    return false;
}

bool MotionPlanner::fixpath_motion_op(Json::Value value, Json::Value &msg)
{
    Json::StyledWriter fast;
    motion_planner::MotionOp srv;

    srv.request.cmd = fast.write(value);
    if(fixpath_nav_client.call(srv))
    {
        //ROS_INFO("fixpath motion op ok!");
        Json::Reader reader;
        ROS_INFO("fixpath motion op:%s",srv.response.result.c_str());
        reader.parse(srv.response.result,msg);
        if(0 == msg[MSG_ERROR_CODE].asInt())
        {
            return true;
        }
        else
        {
            ROS_INFO("operation fixpath motion failed");
        }
    }
    else
    {
        msg[MSG_ERROR_CODE]= Json::Value(100);
        msg[MSG_MSG] = "call fixpath operation failed!";
        ROS_INFO("call fixpath operation failed!");
    }

    return false;
}

//callback cmd_pub topic,
void MotionPlanner::cmd_callback(const std_msgs::String::ConstPtr& msg)
{
    std::string test =msg->data;
    Json::Reader reader;
    Json::Value value;
    if(reader.parse(test,value))
    {
       if(value[MSG_PUB_NAME].isNull())
       {
           ROS_DEBUG("pub_name value is empty");
       }
       else
       {
           //if it is not empty.
           cmd_handle(value);
       }
    }
    else
    {
        ROS_ERROR("reader parse failed!");
    }

}
bool MotionPlanner::lane_following_event_handle(const Json::Value &value)
{
	int result = value["result"].asInt();
	switch(result)
	{
	case 2:
	case 4:
	case 7:
	case 8:
	case 9:
	{
        change_nav_status(NAV_READY);
        current_nav_task = MOVE_IDLE;
        break;
	}


	}

	return true;


}
void MotionPlanner::motion_status_callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("msg:%s",msg->data.c_str());
    std::string test = msg->data;
    Json::Reader reader;
    Json::Value value;
	Json::Value fb_value;
    bool feedback = false;
    int error_code = 0;
    if(reader.parse(test,value))
    {
        if(value.isNull())
        {
            return;
        }
        if(MOVE_OPEN_MOTION == value["planner"].asString())
        {
            if(NAV_READY == value["result"].asInt())
            {
                change_nav_status(NAV_READY);
                current_nav_task = MOVE_IDLE;
                feedback = true;
            }
        }
        else if(MOVE_FIXPATH_MOTION == value["planner"].asString())
        {
            if(NAV_READY == value["result"].asInt())
            {
                change_nav_status(NAV_READY);
                current_nav_task = MOVE_IDLE;
                feedback = true;
            }
            else if(10 == value["result"].asInt())
            {
                change_nav_status(NAV_READY);
                current_nav_task = MOVE_IDLE;
                error_code = 10;
                feedback = true;
            }
            else if(11 == value["result"].asInt())
            {
                change_nav_status(NAV_READY);
                current_nav_task = MOVE_IDLE;
                error_code = 11;
                feedback = true;
            }
        }
        else if(MOVE_MARKER_GUIDE == value["planner"].asString())
        {
            if(NAV_READY == value["result"].asInt())
            {
                change_nav_status(NAV_READY);
                current_nav_task = MOVE_IDLE;
                feedback = true;
            }
        }
        else if(MOVE_LANE_FOLLOWING_MOTION == value["planner"].asString())
        {
        	feedback = lane_following_event_handle(value);
			fb_value[MSG_MSG]="lane following event";
        }
    }
    if(feedback)
    {
        //if handle a valid command of this module,publish feedback.
        std_msgs::String state_msg;
        Json::StyledWriter fast;
        fb_value[MSG_SUB_NAME] = MOTION_PLANNER_EVENT;
        fb_value[MSG_DATA]=value;
        fb_value[MSG_ERROR_CODE] = error_code;
        state_msg.data = fast.write(fb_value);
        state_pub.publish(state_msg);
    }

}
//callback map_list_pub topic,
void MotionPlanner::map_list_callback(const std_msgs::String::ConstPtr& msg)
{
    std::string test =msg->data;
    Json::Reader reader;
    Json::Value value;
    if(reader.parse(test,value))
    {
        if(value.isNull())
        {
            ROS_DEBUG("json value is null");
            return;
        }
        if(!value["current_map_name"].isString())
        {
            ROS_DEBUG("json value is not string");
            return;
        }
        if(0 == value["current_map_name"].asString().size())
        {
            ROS_DEBUG("map list info is not complete");
            map_list_info.current_map = "";
        }
        else
        {
            //if it is not empty.
            map_list_info.current_map = value["current_map_name"].asString();
            //ROS_DEBUG("current_map_name :%s",value["current_map_name"].asString().c_str());
            if(value["map_list"].isNull())
            {
                return;
            }
            Json::Value map_list = value["map_list"];
            if(map_list["mapNames"].isNull() || (!map_list["scene_name"].isString()))
            {
                ROS_DEBUG("map list info is not complete");
            }
            else
            {
                map_list_info.scene = map_list["scene_name"].asString();
                if(map_list["mapNames"].isArray())
                {
                    for(int i=0;i<map_list["mapNames"].size();i++)
                    {
                        //ROS_DEBUG("mapNames[%d]:%s",i,map_list["mapNames"][i].asString().c_str());
                        if(map_list["mapNames"][i].isString())
                        {
                            map_list_info.map_list.push_back(map_list["mapNames"][i].asString());
                        }
                    }
                }

            }
        }
    }

}


//service for other motion planners register
bool MotionPlanner::planners_service(motion_planner::RegisterPlanner::Request &req,
                                     motion_planner::RegisterPlanner::Response &res)
{
    // Planners will call this service and register itself here
    // The service will check whether a planner has been registered.
    // if not, the service will store it, add the planner's name and it' mission client to the planner monitor.
    // else, that's means the planner was dead and restarted. The service will resend the misson which stored
    // in the planner monitor and call map server to resend parameters, if the planner is executing the current navigation task

    int i = 0;
    //ROS_ERROR("service register a module:%s,size:%lu",req.planner.name.c_str(),planners.size());
    for(;i < planners.size();++i)
    {
        //ROS_ERROR("name %s,%s",planners[i].name.c_str(),req.planner.name.c_str());
        if(planners[i].name == req.planner.name)
        {
            //if exist,update state
            planners[i] = req.planner;
            res.error_code = 0;
            //ROS_ERROR("current name %s,%s",current_nav_task.c_str(),req.planner.name.c_str());
            if(this->current_nav_task == req.planner.name)
            {
                //ROS_ERROR("should misson rec");
            	if(this->planner_recover.RecoveryMission(req.planner.name))
            	{
                	motion_planner::MotionOp srv;
                	srv.request.cmd = req.planner.name;
            		param_recovery_client.call(srv);
                    ROS_ERROR("RecoveryMission Succeed,%s",srv.response.result.c_str());

                }
                else
                {
                    ROS_ERROR("RecoveryMission failed");
                }
        	}
            else if(this->current_nav_task == MOVE_FIXPATH_MOTION&&req.planner.name == MOVE_BASE_PLANNER)
            {
                //ROS_ERROR("should misson rec");
                if(this->planner_recover.RecoveryMission(current_nav_task))
                {
                	motion_planner::MotionOp srv;
                	srv.request.cmd = req.planner.name;
            		param_recovery_client.call(srv);
                    ROS_ERROR("RecoveryMission Succeed,%s",srv.response.result.c_str());
                }
                else
                {
                	ROS_ERROR("RecoveryMission failed");
                }
            }
            return true;
        }

    }
    if(i == planners.size())
    {
        //if does not exist,save it.
        planners.push_back(req.planner);

        if(req.planner.name == MOVE_MARKER_GUIDE)
        {
        	this->planner_recover.AddPlannerClient(req.planner.name,&mark_guide_client);
        }
        else if(req.planner.name == MOVE_OPEN_MOTION)
        {
        	this->planner_recover.AddPlannerClient(req.planner.name,&open_motion_client);
        }
        else if(req.planner.name == MOVE_BASE_PLANNER)
        {
            std::string s = "fixpath_motion";
            this->planner_recover.AddPlannerClient(s,&fixpath_nav_client);
        }
        else if(req.planner.name == MOVE_LANE_FOLLOWING_MOTION)
        {
        	this->planner_recover.AddPlannerClient(req.planner.name,&lane_following_motion_client);
        }
        //else if(req.planner.name == MOVE_BASE_PLANNER)
        //{
        //   this->planner_recover.AddPlannerClient(req.planner.name,&lane_following_motion_client);
        //}

        res.error_code = 0;
        return true;
    }
    return false;
}
/*
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server
                      = 10  #gavin add.if robot stuck in place.
*/
//move base motion status callback
void MotionPlanner::movebase_status_callback(const actionlib_msgs::GoalStatusArrayConstPtr& motion_status)
{
    ROS_DEBUG("status list size:%lu",motion_status->status_list.size());
    int i = 0;
    actionlib_msgs::GoalStatus status;
    bool need_publish;

    for(i=motion_status->status_list.size() -1 ;i<motion_status->status_list.size();i++)
    {
        Json::Value msg;
        std_msgs::String state_msg;
        Json::StyledWriter fast;

        need_publish = false;
        status = (motion_status->status_list)[i];
        msg[MSG_SUB_NAME]=CMD_GET_MOTION_STATUS;
        msg[MSG_MSG];
        msg[MSG_DATA];
        msg[MSG_ERROR_CODE] = Json::Value(99);
        switch (status.status)
        {
            case SUCCEEDED:
                ROS_DEBUG("success to get goal");
                if(status.goal_id.id != last_goal_id)
                {
                    need_publish = true;
                    msg[MSG_MSG] =status.goal_id.id + ":" + status.text;
                    msg[MSG_ERROR_CODE] = Json::Value(3);
                    change_nav_status(NAV_READY);
                    current_nav_task = MOVE_IDLE;
                }
                last_goal_id = status.goal_id.id;
                break;
            case PREEMPTED:
                ROS_DEBUG("a new goal received,");
                break;
            case ACTIVE:
                ROS_DEBUG("motion normal");
                break;
            case ABORTED:
                ROS_DEBUG("aborted navigation,something wrong,check costmap");
                need_publish = true;
                msg[MSG_MSG] =status.text+"\n aborted this navigation";
                msg[MSG_ERROR_CODE] = Json::Value(4);
                break;
            case REJECTED:
                ROS_DEBUG("goal was rejected by server,please check goal and costmap");
                need_publish = true;
                msg[MSG_MSG] =status.text;
                msg[MSG_ERROR_CODE] = Json::Value(5);
                break;
            case PREEMPTING:
            case RECALLING:
            case RECALLED:
                ROS_DEBUG("received a cancel request,and have canceled it.");
                need_publish = true;
                msg[MSG_MSG] ="cancel this navigation";
                msg[MSG_ERROR_CODE] = Json::Value(6);
                break;
            case LOST:
                ROS_DEBUG("goal lost");
                break;
            case STUCK:
                ROS_DEBUG("robot stuck and can not find valid path to goal,please help");
                if(STUCK != last_motion_status)
                {
                    need_publish = true;
                    msg[MSG_MSG] = status.text;
                    msg[MSG_ERROR_CODE] = Json::Value(10);
                }
                break;
            case PLAN_SUCCESS:
                ROS_DEBUG("navigation plan path ok!");
                need_publish = true;
                msg[MSG_MSG] = "plan path success!";
                msg[MSG_ERROR_CODE] = Json::Value(11);
                break;
            case PLAN_FAILED:
                ROS_DEBUG("navigation plan path failed!");
                need_publish = true;
                msg[MSG_MSG] = "plan path failed!";
                msg[MSG_ERROR_CODE] = Json::Value(12);
                break;
            case TMP_FIXPATH_STUCK:
                ROS_DEBUG("fixpath navigation stuck!");
                need_publish = true;
                msg[MSG_MSG] = "fixplan path stuck!";
                msg[MSG_ERROR_CODE] = Json::Value(14);
                break;
            case SOME_ROBOT_ON_THE_PATH:
                ROS_DEBUG("SOME_ROBOT_ON_THE_PATH!");
                need_publish = true;
                msg[MSG_MSG] = "SOME_ROBOT_ON_THE_PATH!";
                msg[MSG_ERROR_CODE] = Json::Value(15);
                break;
            default:
                ROS_DEBUG("something error happened");
                break;
        }
        last_motion_status = status.status;

        Json::Value data;
        data["x"] = latest_goal.x;
        data["y"] = latest_goal.y;
        data["th"] = latest_goal.theta;
        data["map"] = latest_goal_map;
        data["name"] = latest_goal_id;
        msg[MSG_DATA] = data;

        if(need_publish)
        {
            state_msg.data = fast.write(msg);
            state_pub.publish(state_msg);
        }
    }
}

