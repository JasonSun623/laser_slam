#include <libgen.h>
#include "localization/localization.h"

//localization app pub cmd 
static const char* MAP_BUILD = "map_build";
static const char* MAP_BUILD_FINISH = "map_build_finish";
static const char* MAP_LOAD_FINISH = "map_load_finish";
static const char* MAP_MODULES_GET = "map_modules_get";
static const char* MAP_MODULE_CURRENT = "map_module_current";
static const char* MAP_MODULE_SET = "map_module_set";
static const char* NAV_MODULES_GET = "nav_modules_get";
static const char* NAV_MODULE_CURRENT = "nav_module_current";
static const char* NAV_MODULE_SET = "nav_module_set";
static const char* AMCL_PARAMS_GET = "amcl_params_get";
static const char* AMCL_PARAM_SET = "amcl_param_set";
static const char* GMAPPING_PARAMS_GET = "gmapping_params_get";
static const char* GMAPPING_PARAM_SET = "gmapping_param_set";
static const char* UWB_OPEN = "uwb_open";
static const char* UWB_CLOSE = "uwb_close";
static const char* ROBOT_POSE_SET = "robot_pose_set";

//return rlt str & int
static const char* SUCCESS_STR = "success";
static const char* FAIL_STR = "arguments invalid";
static const int ERRORCODE_SUCCESS = 0;
static const int ERRORCODE_FAIL = 1;

// localization topic & service
static const char* APP_PUB_TOPIC = "app_pub";
static const char* APP_SUB_TOPIC = "app_sub";
static const char* GET_STATE_SERVICE = "get_localizaiton_state";
static const char* REGISTER_OK_SERVICE= "register_localizaiton_ok";
static const char* CURRENT_POSE= "current_pose";
static const char* INITIAL_POSE_TOPIC = "initialpose";
static const char* ODOM = "odom";
static const char* IS_USE_UWB = "is_use_uwb";

// param type
static const std::string INT = "int";
static const std::string BOOL = "bool";
static const std::string DOUBLE = "double";
static const std::string STRING = "string";

static const int INITIAL_POSE_DURATION = 60*60;
static const int TYPE_1 = 1;
static const int TYPE_2 = 2;

LocalizationNode::LocalizationNode():priv_nh("~")
{
    tf_listener = new tf::TransformListener();
    localization_sub = nh.subscribe(APP_PUB_TOPIC, 1000, &LocalizationNode::appPubCallback,this);
    localization_pub = nh.advertise<std_msgs::String>(APP_SUB_TOPIC, 1000);
    odom_sub = nh.subscribe(ODOM, 1, &LocalizationNode::pubCurrentPose, this);
    pose_pub = nh.advertise<nav_msgs::Odometry>(CURRENT_POSE, 2, true);
    initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(INITIAL_POSE_TOPIC, 1000);
    state_service = priv_nh.advertiseService(GET_STATE_SERVICE, &LocalizationNode::getState, this);
    register_service = priv_nh.advertiseService(REGISTER_OK_SERVICE, &LocalizationNode::registerLocalizationOK, this);
    register_system_thread = new boost::thread(boost::bind(&LocalizationNode::registerSystemThread, this));
    register_client = nh.serviceClient<robot_state_keeper::RegisterState>("/robot_state_keeper/register_robot_modules");
    // set state with defult module
    setState(localizationModule.getDefaultModule());
    if(!priv_nh.getParam("cov_x_1", cov_x_1))
        cov_x_1 = 0.25;
    if(!priv_nh.getParam("cov_y_1", cov_y_1))
        cov_y_1 = 0.25;
    if(!priv_nh.getParam("cov_th_1", cov_th_1))
        cov_th_1 = 0.068;
    if(!priv_nh.getParam("cov_x_2", cov_x_2))
        cov_x_2 = 1;
    if(!priv_nh.getParam("cov_y_2", cov_y_2))
        cov_y_2 = 1;
    if(!priv_nh.getParam("cov_th_2", cov_th_2))
        cov_th_2 = 0.25;
    pub_current_pose_count = 0;
}

LocalizationNode::~LocalizationNode()
{
    ROS_DEBUG("LocalizationNode class over");
    // interupt register_system thread
    register_system_thread->interrupt();
    register_system_thread->join();

    delete register_system_thread;
    delete tf_listener;
}

/*
*  get amcl state or get gmapping state 
*/
bool LocalizationNode::getState(localization::localization_state::Request  &req,
                                localization::localization_state::Response &res)
{
   res.state = getState();
   return true;
}

/*
*  register amcl ok or register gmapping ok 
*/
bool LocalizationNode::registerLocalizationOK(localization::localization_register::Request &req,
              localization::localization_register::Response &res)
{
   localizationModule.registerModule(req.name);
   return true;
}


/*
*  from app pub cmd set amcl state or set gmapping state 
*  send app sub success or fail
*/
void LocalizationNode::appPubCallback(const std_msgs::String::ConstPtr& msg)
{     
     std::string cmdName;
     if(localizationJson.readJsonStr(msg))
     {
          cmdName = localizationJson.getAppPubName();
          ROS_INFO("app pub name:%s",cmdName.c_str());
          handleCmd(cmdName);
     }
}

/*
* get state
*/
std::string LocalizationNode::getState()
{
    return state;
}

/*
* set current state
*/
int LocalizationNode::setState(std::string currentState)
{
    state = currentState;
    return 0;
}

/*
*  registerSystem localizationModule
*/
void LocalizationNode::registerSystem()
{
   if(localizationModule.isModulesOK())
   {
       motion_state.enabled = 0;
       //ROS_INFO("Localization all module OK");
   }
   else
   {
       motion_state.enabled = 1;
       //ROS_INFO("Localization some modules fail");
   }
   motion_state.name = "localization";
   motion_state.error_code = 0;
   motion_state.info = "robot localization control center";  
   robot_state_keeper::RegisterState srv;
   srv.request.state = motion_state;
   register_client.call(srv);
}

/*
*  register system state thread
*/
void LocalizationNode::registerSystemThread()
{
   ros::Rate loop_rate(0.5);
   ROS_DEBUG("register system thread start");
   while(ros::ok())
   {
      registerSystem();
      ros::spinOnce();
      loop_rate.sleep();
   }
}

/*
*  handleCmd : map_build  map_build_finish  map_load_finish ...
*/
void LocalizationNode::handleCmd(std::string cmdName)
{ 
    int rlt;
    std::string rltStr;
    Json::Value sendData;
    Json::Value revData;
    std::string paramName;
    bool sendflag = false;
    // cmd : map_build
    if(0 == cmdName.compare(MAP_BUILD))
    {            
        std::string mapfun = localizationModule.getCurrentMapModule();
        rlt = (0 != mapfun.compare(""))?setState(mapfun):ERRORCODE_FAIL;
        sendflag = true;
    }
    // cmd : map_build_finish
    else if(0 == cmdName.compare(MAP_BUILD_FINISH))
    {
        std::string navfun = localizationModule.getCurrentNavModule();
        rlt = (0 != navfun.compare(""))?setState(navfun):ERRORCODE_FAIL;
        sendflag = true;
    }
    // cmd : map_load_finish
    else if(0 == cmdName.compare(MAP_LOAD_FINISH))
    {
        std::string navfun = localizationModule.getCurrentNavModule();
        rlt = (0 != navfun.compare(""))?setState(navfun):ERRORCODE_FAIL;
        sendflag = true;
    }
    // cmd : map_modules_get
    else if(0 == cmdName.compare(MAP_MODULES_GET))
    {
        rltStr = localizationModule.queryMapModule();
        rlt = (0 == rltStr.compare("")) ? ERRORCODE_FAIL:ERRORCODE_SUCCESS;
        sendData["map_module"] = rltStr;
        sendflag = true;
    }
    // cmd : map_module_current
    else if(0 == cmdName.compare(MAP_MODULE_CURRENT))
    {
        rltStr = localizationModule.getCurrentMapModule();
        rlt = (0 == rltStr.compare("")) ? ERRORCODE_FAIL:ERRORCODE_SUCCESS;
        sendData["map_use"] = rltStr;
        sendflag = true;
    }
    // cmd : map_module_set
    else if(0 == cmdName.compare(MAP_MODULE_SET))
    {
        std::string setModule;
        revData = localizationJson.getAppPubData();
        // receive data is not null and map_use is string type
        if(!revData.isNull())
        {
            if(revData["map_use"].isString())
            {
                 setModule = revData["map_use"].asString();
            }
            rlt = (0 != setModule.compare(""))?
                  localizationModule.setCurrentMapModule(setModule):ERRORCODE_FAIL;
        }
        sendflag = true;
    }
    // cmd : nav_modules_get
    else if(0 == cmdName.compare(NAV_MODULES_GET))
    {
        rltStr = localizationModule.queryNavModule();
        rlt = (0 == rltStr.compare("")) ? ERRORCODE_FAIL:ERRORCODE_SUCCESS;
        sendData["nav_module"] = rltStr;
        sendflag = true;
    }
    // cmd : nav_module_current
    else if(0 == cmdName.compare(NAV_MODULE_CURRENT))
    {
        rltStr = localizationModule.getCurrentNavModule();
        rlt = (0 == rltStr.compare("")) ? ERRORCODE_FAIL:ERRORCODE_SUCCESS;
        sendData["nav_use"] = rltStr;
        sendflag = true;
    }
    // cmd : nav_module_set
    else if(0 == cmdName.compare(NAV_MODULE_SET))
    {
        std::string setModule;
        revData = localizationJson.getAppPubData();
        // receive data is not null and map_use is string type
        if(!revData.isNull())
        {
            if(revData["map_use"].isString())
            {
                setModule = revData["nav_use"].asString();
            }  
            rlt = (0 != setModule.compare(""))?
                  localizationModule.setCurrentNavModule(setModule):ERRORCODE_FAIL;
        }
        sendflag = true;
    }
    // cmd : amcl_params_get
    else if(0 == cmdName.compare(AMCL_PARAMS_GET))
    {
        sendData = getAmclParams();
        rlt = (sendData.isNull()) ? ERRORCODE_FAIL:ERRORCODE_SUCCESS;
        sendflag = true;
    }
    // cmd : amcl_param_set
    else if(0 == cmdName.compare(AMCL_PARAM_SET))
    {
        revData = localizationJson.getAppPubData();
        // receive data is not null and map_use is string type
        if(!revData.isNull())
        {
            if(revData["name"].isString())
            {
                paramName = revData["name"].asString();
            }
            rlt = ((0 != paramName.compare("")) && (!revData["value"].isNull()))?
                  localizationModule.getAmclParamManager().setSubParamValue(paramName,revData["value"])
                  :ERRORCODE_FAIL;
        }
        sendflag = true;
    }
    // cmd : gmapping_params_get
    else if(0 == cmdName.compare(GMAPPING_PARAMS_GET))
    {
        sendData = getGmappingParams();
        rlt = (sendData.isNull()) ? ERRORCODE_FAIL:ERRORCODE_SUCCESS;
        sendflag = true;
    }
    // cmd : gmapping_param_set
    else if(0 == cmdName.compare(GMAPPING_PARAM_SET))
    {
        revData = localizationJson.getAppPubData();
        // receive data is not null and map_use is string type
        if(!revData.isNull())
        {
            if(revData["name"].isString())
            {
                paramName = revData["name"].asString();
            }
            rlt = (0 != paramName.compare(""))?
                  localizationModule.getGmappingParamManager().setSubParamValue(paramName,revData["value"])
                  :ERRORCODE_FAIL;
        }
        sendflag = true;
    }
    // cmd : uwb_open
    else if(0 == cmdName.compare(UWB_OPEN))
    {
        std::string name = IS_USE_UWB;
        Json::Value tmp = Json::Value(true);
        rlt = localizationModule.getUwbParamManager().setSubParamValue(name,tmp);
        sendflag = true;
    }
    // cmd : uwb_close
    else if(0 == cmdName.compare(UWB_CLOSE))
    {
        std::string name = IS_USE_UWB;
        Json::Value tmp = Json::Value(false);
        rlt = localizationModule.getUwbParamManager().setSubParamValue(name,tmp);
        sendflag = true;
    }
    //cmd set robot pose
    else if(0 == cmdName.compare(ROBOT_POSE_SET))
    {
        rlt = setInitialPose(localizationJson.getAppPubData());
        sendflag = true;
    }
    
    // send app msg
    if(sendflag)
    {
        sendMsg(rlt,cmdName,sendData);
    }
}

/*
*  send Msg
*/
void LocalizationNode::sendMsg(int rlt, std::string cmdName, Json::Value data)
{
     std_msgs::String send_msg;
     if(rlt == ERRORCODE_SUCCESS)
     {
         send_msg = localizationJson.writeJsonStr(cmdName,ERRORCODE_SUCCESS,SUCCESS_STR,data);
     }
     else
     {
         send_msg = localizationJson.writeJsonStr(cmdName,ERRORCODE_FAIL,FAIL_STR,data);
     }
     localization_pub.publish(send_msg);
}

/*
*  get all amcl params
*/
Json::Value LocalizationNode::getAmclParams()
{
    std::vector<SubParamValue> amclParamVector = localizationModule.getAmclParamManager().getSubParamsVector();
    return getSubParams(amclParamVector);
}

/*
*  get all gmapping params
*/
Json::Value LocalizationNode::getGmappingParams()
{
    std::vector<SubParamValue> gmappingParamVector = localizationModule.getGmappingParamManager().getSubParamsVector();
    return getSubParams(gmappingParamVector);
}

/*
*  get all gmapping params
*/
Json::Value LocalizationNode::getSubParams(std::vector<SubParamValue> subParamVector)
{
    Json::Value data;
    Json::Value item;
    for(std::vector<SubParamValue>::iterator it = subParamVector.begin(); it != subParamVector.end(); ++ it)
    {
        item["name"] = Json::Value((*it).name);
        try{
            if(0 == (*it).type.compare(INT))
            {
                item["value"] = Json::Value(boost::any_cast<int>((*it).value));
            }
            else if(0 == (*it).type.compare(DOUBLE))
            {
                item["value"] = Json::Value(boost::any_cast<double>((*it).value)); 
            }
            else if(0 == (*it).type.compare(BOOL))
            {
                item["value"] = Json::Value(boost::any_cast<bool>((*it).value));
            }
            else if(0 == (*it).type.compare(STRING))
            {
                item["value"] = Json::Value(boost::any_cast<std::string>((*it).value));
            }
        }catch(boost::bad_any_cast e){
            return data;
        }
        
        data.append(item);
    }
    return data;
}

/*
*  pub current pose
*/
void LocalizationNode::pubCurrentPose(const nav_msgs::Odometry::ConstPtr& odom_msg)
{ 
    nav_msgs::Odometry map_pose;
    geometry_msgs::PoseStamped msg_pose;
    tf::Stamped<tf::Pose> tmp_pose;
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), 
                                 tf::Vector3(0,0,0)), ros::Time(0), "base_link");
    try  
    {  
        tf_listener -> transformPose("map", ident, tmp_pose);
    }  
    catch(tf::TransformException e)
    {  
        ROS_DEBUG("Failed to compute current pose, skipping scan (%s)", e.what());  
        return;  
    }  
    tf::poseStampedTFToMsg(tmp_pose, msg_pose);
    map_pose.header.stamp = ros::Time::now();
    map_pose.header.frame_id = "map";
    //set the position
    map_pose.pose.pose.position.x = msg_pose.pose.position.x;
    map_pose.pose.pose.position.y = msg_pose.pose.position.y;
    map_pose.pose.pose.position.z = 0.0;
    map_pose.pose.pose.orientation =msg_pose.pose.orientation;
    //set the velocity
    map_pose.child_frame_id = "base_link";
    map_pose.twist.twist.linear.x = odom_msg->twist.twist.linear.x;
    map_pose.twist.twist.linear.y = 0.0;
    map_pose.twist.twist.angular.z = odom_msg->twist.twist.angular.z;

    if(pub_current_pose_count >= 10)
    {
        pose_pub.publish(map_pose);
        pub_current_pose_count = 0;
    }
    else
    {
        pub_current_pose_count ++;
    }
}

/*
*  set robot pose
*/
int LocalizationNode::setInitialPose(Json::Value revData)
{
    int rlt;
    //(ros::Time::now().toSec()-(revData["time"].asDouble())<INITIAL_POSE_DURATION)
    if(!revData.isNull() && revData["type"].isInt() && revData["time"].isDouble() &&
            revData["x"].isDouble() && revData["y"].isDouble() && revData["th"].isDouble())
    {
        initial_pose.header.frame_id = "map";
        initial_pose.header.stamp = ros::Time::now();
        initial_pose.pose.pose.position.x = revData["x"].asDouble();
        initial_pose.pose.pose.position.y = revData["y"].asDouble();
        initial_pose.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(revData["th"].asDouble());
        initial_pose.pose.pose.orientation = odom_quat;
        double cov_x,cov_y,cov_th;
        if(TYPE_1 == revData["type"].asInt())
        {
            cov_x = cov_x_1;
            cov_y = cov_y_1;
            cov_th = cov_th_1;
        }
        else if (TYPE_2 == revData["type"].asInt())
        {
            cov_x = cov_x_2;
            cov_y = cov_y_2;
            cov_th = cov_th_2;
        }
        initial_pose.pose.covariance = boost::assign::list_of
                                (cov_x) (0)  (0)  (0)  (0)  (0)
                                (0)  (cov_y) (0)  (0)  (0)  (0)
                                (0)  (0)  (0)  (0)  (0)  (0)
                                (0)  (0)  (0)  (0)  (0)  (0)
                                (0)  (0)  (0)  (0)  (0)  (0)
                                (0)  (0)  (0)  (0)  (0)  (cov_th);
        initial_pose_pub.publish(initial_pose);
        rlt = ERRORCODE_SUCCESS;
    }
    else
    {
        rlt = ERRORCODE_FAIL;
    }
    return rlt;
}

/*
*  main
*/
int main(int argc, char **argv)
{ 
   ros::init(argc, argv, "localization");
   LocalizationNode localizationNode;
   ros::spin();
   return 0;
}
