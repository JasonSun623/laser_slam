/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
//#include <move_base/move_base.h>
#include "move_base/move_base.h"
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include "motion_planner/RegisterPlanner.h"
namespace move_base
{

    MoveBase::MoveBase(tf::TransformListener& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    fgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), 
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false),runFixpathPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false),motion_pause_(false),
    has_recovery_all_place_(false),fixpath_goal_reached_(false),fixpath_status(NAV_READY),fixpath_goal_tolerance_type_(0)
    {

    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    fixpath_nav_srv = nh.advertiseService("/fixpath_nav_op",&MoveBase::fixpath_nav_service,this);

    fixpath_plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    fixpath_motion_status_pub_ = private_nh.advertise<std_msgs::String>("/motion_status",30);

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner,fixpath_global_plaanner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));//"navfn/NavfnROS"
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("fixpath_global_planner", fixpath_global_plaanner, std::string("fixpath_global_planner/FixpathGlobalPlanner"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, 3);  // -1 :disabled by default,gavin modify to 3

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);
    private_nh.param("wait_all_clear_times", wait_all_clear_times_, 5);
    private_nh.param("config_file_path",config_file_path_,std::string("/home/robot/catkin_ws/src/navigation/move_base/configs/"));
    private_nh.param("in_line_distance",in_line_distance_,2.0);
    private_nh.param("on_path_distance",on_path_distance_,0.5);
    private_nh.param("robot_id",self_id_,std::string("NULL"));
    get_config_param_file_ = false;
    std::string com_costmap_file = config_file_path_ + "costmap_common_params.yaml";
    std::string global_costmap_file = config_file_path_ + "moverglobal.yaml";
    std::string local_costmap_file = config_file_path_ + "moverlocal.yaml";
    std::string planner_info_file = config_file_path_ + "moverlocalplanner.yaml";

    std::ifstream com_costmap_fin(com_costmap_file.c_str());
    std::ifstream global_costmap_fin(global_costmap_file.c_str());
    std::ifstream local_costmap_fin(local_costmap_file.c_str());
    std::ifstream planner_info_fin(planner_info_file.c_str());
    if(com_costmap_fin && global_costmap_fin && local_costmap_fin && planner_info_fin)
    {
        com_costmap_info_ = YAML::Load(com_costmap_fin);
        global_costmap_info_ = YAML::Load(global_costmap_fin);
        local_costmap_info_ = YAML::Load(local_costmap_fin);
        planner_info_ = YAML::Load(planner_info_fin);

        if(com_costmap_info_ && global_costmap_info_ && local_costmap_info_ && planner_info_)
        {
            get_config_param_file_ = true;
        }
    }
    com_costmap_fin.close();
    global_costmap_fin.close();
    local_costmap_fin.close();
    planner_info_fin.close();

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //set up fixpath planner's thread
    fixpath_planner_thread_ = new boost::thread(boost::bind(&MoveBase::fixpathPlanThread, this));

    //for comanding the base
    //vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("nav_pri_cmdvel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    motion_status_pub_ = action_nh.advertise<actionlib_msgs::GoalStatusArray>("status",1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, false);  //gavin:modify to false.
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

	current_pose_sub = nh.subscribe("current_pose", 2, &MoveBase::current_pose_callback, this);
    robots_info_sub_ = nh.subscribe("/agent_pub",10,&MoveBase::robots_info_callback,this);

	ROS_ERROR("global_planner: %s",global_planner.c_str());
    //initialize the global planner
    try
    {
      //check if a non fully qualified name has potentially been passed in
      if(!bgp_loader_.isClassAvailable(global_planner))
      {
        std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i)
        {
          if(global_planner == bgp_loader_.getName(classes[i]))
          {
            //if we've found a match... we'll get the fully qualified name and break out of the loop
            ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                global_planner.c_str(), classes[i].c_str());
            global_planner = classes[i];
            break;
          }
        }
      }

      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    ROS_ERROR("fixpath_global_planner: %s",fixpath_global_plaanner.c_str());
    //initialize the fixpath global planner
    try
    {
      if(!fgp_loader_.isClassAvailable(fixpath_global_plaanner))
      {
        std::vector<std::string> classes = fgp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i)
        {
			ROS_ERROR("class name: %s",fgp_loader_.getName(classes[i]).c_str());
          if(fixpath_global_plaanner == fgp_loader_.getName(classes[i]))
          {
            ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                fixpath_global_plaanner.c_str(), classes[i].c_str());
            fixpath_global_plaanner = classes[i];
            break;
          }
        }
      }

      ROS_ERROR("before initial fix golbal planner");
      fixpath_planner_ = fgp_loader_.createInstance(fixpath_global_plaanner);
      fixpath_planner_->initialize(fgp_loader_.getName(fixpath_global_plaanner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", fixpath_global_plaanner.c_str(), ex.what());
      exit(1);
    }/**/

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try
    {
      //check if a non fully qualified name has potentially been passed in
      if(!blp_loader_.isClassAvailable(local_planner))
      {
        std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i)
        {
          if(local_planner == blp_loader_.getName(classes[i]))
          {
            //if we've found a match... we'll get the fully qualified name and break out of the loop
            ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                local_planner.c_str(), classes[i].c_str());
            local_planner = classes[i];
            break;
          }
        }
      }

      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //motion_pause_
    motion_control_srv_ = private_nh.advertiseService("motion_control", &MoveBase::pauseService, this);

    //pause costmap update
    costmap_pause_update_srv_ = private_nh.advertiseService("costmap_pause_update",&MoveBase::costmapPauseUpdateService,this);

    //configure param read and write
    config_param_srv_ = private_nh.advertiseService("configure_param",&MoveBase::configParamService,this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh))
    {
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    as_->start();

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    ros::NodeHandle motion_planner_nh("motion_planner");
    planner_client = motion_planner_nh.serviceClient<motion_planner::RegisterPlanner>("planners_register");

    move_base_planner_state.enabled = 0;
    move_base_planner_state.name = "move_base";
    move_base_planner_state.info = "move base could calculate velocity based on costmap";
    move_base_planner_state.error_code = 0;

    register_thread_ = new boost::thread(boost::bind(&MoveBase::registerThread, this));

    /*bool registered = false;
    ros::Rate r(1);
    while(ros::ok()&&(!registered))
    {
        //call service
        motion_planner::RegisterPlanner srv;
        srv.request.planner = move_base_planner_state;
        if(planner_client.call(srv))
        {
            ROS_INFO("motion planner register ok!");
            registered = true;
        }
        else
        {
            ROS_INFO("motion planner register failed!");
        }
        r.sleep();
    }*/
  }

void MoveBase::robots_info_callback(const std_msgs::String& msg)
{
    Json::Value res;
    Json::Reader reader;
	//ROS_INFO("%s",msg.data.c_str());
    if(reader.parse(msg.data,res))
    {
        if(res[MSG_PUB_NAME].isNull())
        {
            ROS_INFO("pub_name value is empty");
        }
        else
        {
            if(ROBOTS_CURRENT_POSE == res[MSG_PUB_NAME].asString())
            {
                self_id_ = res["target_robot_code"].asString();
                std::vector<robot_info_t> infos;
                Json::Value data = res[MSG_DATA];
                for(int i=0;i < data.size();++i)
                {
					Json::Value robot_i = data[i];
                    robot_info_t info;
                    info.robot_id = robot_i["code"].asString();

					if(info.robot_id == self_id_) continue;

                    info.scene_name = robot_i["scene_name"].asString();
                    info.map_name = robot_i["map_name"].asString();
					Json::Value pose_i = robot_i["pose"]["pose"];
					Json::Value twist_i = robot_i["twist"]["twist"];
                    info.pose.pose.pose.position.x = pose_i["position"]["x"].asDouble();
                    info.pose.pose.pose.position.y = pose_i["position"]["y"].asDouble();
                    info.pose.pose.pose.orientation.x = pose_i["orientation"]["x"].asDouble();
                    info.pose.pose.pose.orientation.y = pose_i["orientation"]["y"].asDouble();
                    info.pose.pose.pose.orientation.z = pose_i["orientation"]["z"].asDouble();
                    info.pose.pose.pose.orientation.w = pose_i["orientation"]["w"].asDouble();	
                    info.pose.twist.twist.linear.x = twist_i["linear"]["x"].asDouble();
                    info.pose.twist.twist.linear.y = twist_i["linear"]["y"].asDouble();
                    info.pose.twist.twist.linear.z = twist_i["linear"]["z"].asDouble();
                    info.pose.twist.twist.angular.x = twist_i["angular"]["x"].asDouble();
                    info.pose.twist.twist.angular.y = twist_i["angular"]["y"].asDouble();
                    info.pose.twist.twist.angular.z = twist_i["angular"]["z"].asDouble();
                    infos.push_back(info);
                }
                if(!robots_info_.empty())
                {
                    robots_info_.clear();
                }/*
				for(int i = 0; i < infos.size();++i)
				{
					robot_info_t info = infos[i];
					ROS_DEBUG("%s,%s,%s",info.robot_id.c_str(),info.scene_name.c_str(),info.map_name.c_str());
					ROS_DEBUG("%f,%f,%f,%f",info.pose.pose.pose.position.x,info.pose.pose.pose.position.y,info.pose.pose.pose.orientation.z,info.pose.pose.pose.orientation.w);
					ROS_DEBUG("%f,%f,%f",info.pose.twist.twist.linear.x,info.pose.twist.twist.linear.y,info.pose.twist.twist.angular.z);
                    infos.push_back(info);
				}*/
				boost::mutex::scoped_lock lock(robots_pose_mutex_);	
                robots_info_ = infos;
            }
        }
    }
}

void MoveBase::current_pose_callback(const nav_msgs::Odometry& msg)
{
	cur_pose.pose = msg.pose.pose;
    current_pose_ = msg;
}

Json::Value MoveBase::HandleCmd(Json::Value cmd)
{
    Json::Value res;
    res[MSG_SUB_NAME] = cmd[MSG_PUB_NAME];
    ROS_INFO("recevied a fixpath motion cmd:%s",cmd[MSG_PUB_NAME].asString().c_str());
    if(CMD_FIXPATH_MOTION_PAUSE == cmd[MSG_PUB_NAME].asString())
    {
        //pause;
        if((NAV_MOVING == fixpath_status) || (NAV_PAUSE == fixpath_status))
        {
            fixpath_status = NAV_PAUSE;
            res[MSG_ERROR_CODE] = 0;
            res[MSG_MSG] = "pause success";
        }
        else
        {
            res[MSG_ERROR_CODE] = 1;
            res[MSG_MSG] = "pause failed status is not right";
        }
    }
    else if(CMD_FIXPATH_MOTION_RESUME == cmd[MSG_PUB_NAME].asString())
    {
        //resume pause;
        if((NAV_MOVING == fixpath_status) || (NAV_PAUSE == fixpath_status))
        {
            fixpath_status = NAV_MOVING;
            res[MSG_ERROR_CODE] = 0;
            res[MSG_MSG] = "resume success";
        }
        else
        {
            res[MSG_ERROR_CODE] = 1;
            res[MSG_MSG] = "resume failed status is not right";
        }/**/
    }
    else if(CMD_FIXPATH_MOTION_START == cmd[MSG_PUB_NAME].asString())
    {
        Json::Value data;
        data = cmd[MSG_DATA];
        if(data["id"].isInt() && data["scene_name"].isString() && data["map_name"].isString())
        {
            cur_fixpath_id_ = data["id"].asInt();
            cur_fixpath_scene_name_ = data["scene_name"].asString();
	    cur_fixpath_map_name_ = data["map_name"].asString();
            if(data["tolerance_type"].isInt())
            {
                fixpath_goal_tolerance_type_ = data["tolerance_type"].asInt();
            }
            else
            {
                fixpath_goal_tolerance_type_ = 0;
            }
            ros::NodeHandle n("~");
            double xy_dist,th_dist;
            if(0 == (fixpath_goal_tolerance_type_%100))
            {
                xy_dist = 0.1;
                th_dist = 0.05;
            }
            else if(1 == (fixpath_goal_tolerance_type_%100))
            {
                xy_dist = 0.05;
                th_dist = 0.1;
            }
            else if(10 == (fixpath_goal_tolerance_type_%100))
            {
                xy_dist = 0.5;
                th_dist = 3.14;
            }
            else if(11 == (fixpath_goal_tolerance_type_%100))
            {
                xy_dist = 0.1;
                th_dist = 3.14;
            }
            else
            {
                xy_dist = 0.1;
                th_dist = 3.14;
            }

            n.setParam("xy_goal_tolerance", xy_dist);
            n.setParam("yaw_goal_tolerance", th_dist);

            fixpath_status = NAV_MOVING;
            runFixpathPlanner_ = true;
            fixpath_planner_cond_.notify_one();
            res[MSG_ERROR_CODE]= Json::Value(0);
            res[MSG_MSG];
        }
        else
        {
            res[MSG_ERROR_CODE]= Json::Value(1);
            res[MSG_MSG]="cmd data error";
        }

    }
    else if(CMD_FIXPATH_MOTION_CANCEL == cmd[MSG_PUB_NAME].asString())
    {
        geometry_msgs::Twist twist;
        //cancel a marker guide;
        if((NAV_MOVING == fixpath_status) || (NAV_PAUSE == fixpath_status))
        {
            fixpath_status = NAV_READY;
            runFixpathPlanner_ = false;

            res[MSG_ERROR_CODE] = 0;
            res[MSG_MSG] = "cancel success";
            publishZeroVelocity();
        }
        else
        {
            res[MSG_ERROR_CODE] = 1;
            res[MSG_MSG] = "cancel failed status is not right";
        }
    }
    return res;
}

bool MoveBase::fixpath_nav_service(motion_planner::MotionOp::Request &req,motion_planner::MotionOp::Response &res)
{

    std::string test =req.cmd;
    Json::Reader reader;
    Json::Value value;
    Json::Value result;
    ROS_INFO("fixpath motion:%s",test.c_str());
    if(reader.parse(test,value))
    {
       if(value[MSG_PUB_NAME].isNull())
       {
           ROS_INFO("pub_name value is empty");
       }
       else
       {
           //if it is not empty.
           result = HandleCmd(value);
           Json::StyledWriter fast;
           res.result = fast.write(result);
           res.error_code = 0;
           return true;
       }
    }
    res.result = "op failed!";
    res.error_code = 1;
    return false;
}

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level)
  {
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults)
    {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner)
    {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try
      {
        //check if a non fully qualified name has potentially been passed in
        if(!bgp_loader_.isClassAvailable(config.base_global_planner))
        {
          std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
          for(unsigned int i = 0; i < classes.size(); ++i)
          {
            if(config.base_global_planner == bgp_loader_.getName(classes[i]))
            {
              //if we've found a match... we'll get the fully qualified name and break out of the loop
              ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                  config.base_global_planner.c_str(), classes[i].c_str());
              config.base_global_planner = classes[i];
              break;
            }
          }
        }

        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner)
    {
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try
      {
        //check if a non fully qualified name has potentially been passed in
        ROS_INFO("Loading local planner: %s", config.base_local_planner.c_str());
        if(!blp_loader_.isClassAvailable(config.base_local_planner))
        {
          std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
          for(unsigned int i = 0; i < classes.size(); ++i)
          {
            if(config.base_local_planner == blp_loader_.getName(classes[i]))
            {
              //if we've found a match... we'll get the fully qualified name and break out of the loop
              ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                  config.base_local_planner.c_str(), classes[i].c_str());
              config.base_local_planner = classes[i];
              break;
            }
          }
        }
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    last_config_ = config;
  }

  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y)
  {
    tf::Stamped<tf::Pose> global_pose;

    //clear the planner's costmap
    planner_costmap_ros_->getRobotPose(global_pose);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    controller_costmap_ros_->getRobotPose(global_pose);

    clear_poly.clear();
    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();

    pt.x = x - size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    //clear the costmaps
    planner_costmap_ros_->resetLayers();
    controller_costmap_ros_->resetLayers();
    return true;
  }


  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
  {
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(as_->isActive())
    {
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL)
    {
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose))
    {
      ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
      return false;
    }
    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id == "")
      tf::poseStampedTFToMsg(global_pose, start);
    else
      start = req.start;

    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    //first try to make a plan to the exact desired goal
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  bool MoveBase::pauseService(Pause::Request &req, Pause::Response &resp)
  {
      motion_pause_ = req.pause;
      ROS_INFO("move base motion pause:%s",motion_pause_ ? "true":"false");
      resp.error_code = 0;
      return true;
  }

  double MoveBase::get_double_value(Json::Value value,std::string item)
  {
      double tmp = -1.0;
      if(value[item.c_str()].isDouble() )
      {
          tmp = value[item.c_str()].asDouble();
      }
      else if(value[item.c_str()].isInt())
      {
          tmp = value[item.c_str()].asInt();
      }
      return tmp;
  }

  bool MoveBase::getAndSetConfigParam(Json::Value value,Json::Value &feedback)
  {
      bool need_update_config_file = false;
      double tmp = 0.0;
      feedback = value;
      if(value[CONFIG_PARAM_SPACE].asString() == COM_COSTMAP)
      {
          if(value[CONFIG_PARAM_NAME].asString() == PARAM_RAYTACE_RANGE)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify raytrace range
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      com_costmap_info_[PARAM_RAYTACE_RANGE] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = com_costmap_info_[PARAM_RAYTACE_RANGE].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_OBSTACLE_RANGE)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify obstacle range
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      com_costmap_info_[PARAM_OBSTACLE_RANGE] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = com_costmap_info_[PARAM_OBSTACLE_RANGE].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_ROBOT_RADIUS)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify robot radius
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      com_costmap_info_[PARAM_ROBOT_RADIUS] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = com_costmap_info_[PARAM_ROBOT_RADIUS].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_FOOTPRINT_ENABLE)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify footprint enable
                  if(value[CONFIG_PARAM_VALUE].isBool())
                  {
                      com_costmap_info_[PARAM_FOOTPRINT_ENABLE] = value[CONFIG_PARAM_VALUE].asBool();
                      need_update_config_file = true;
                  }
                  else
                  {
                      feedback[CONFIG_MSG] = "parm value is wrong";
                      return false;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = com_costmap_info_[PARAM_FOOTPRINT_ENABLE].as<bool>();
              }
          }
          if(need_update_config_file)
          {
              std::string com_costmap_file = config_file_path_ + "costmap_common_params.yaml";
              std::ofstream fout(com_costmap_file.c_str());
              if(!fout)
              {
                  feedback[CONFIG_MSG] = "could not open configure file";
                  return false;
              }
              fout << com_costmap_info_;
              fout.close();
          }
      }//com costmap configure param
      else if(value[CONFIG_PARAM_SPACE].asString() == PARAM_GLOBAL_COSTMAP)
      {
          YAML::Node global = global_costmap_info_[PARAM_GLOBAL_COSTMAP];

          if(value[CONFIG_PARAM_NAME].asString() == PARAM_UPDATE_FREQ)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify update frequency
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp < 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      global[PARAM_UPDATE_FREQ] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = global[PARAM_UPDATE_FREQ].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_PUBLISH_FREQ)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify publish frequency
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp < 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      global[PARAM_PUBLISH_FREQ] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = global[PARAM_PUBLISH_FREQ].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_INFLATION_RADIUS)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify inflation radius
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      global[PARAM_INFLATION_RADIUS] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = global[PARAM_INFLATION_RADIUS].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_COST_SCALING_FACTOR)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify cost_scaling_factor
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      global[PARAM_COST_SCALING_FACTOR] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = global[PARAM_COST_SCALING_FACTOR].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_FORBIDDEN_RADIUS)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify forbidden_radius
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      global[PARAM_FORBIDDEN_RADIUS] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = global[PARAM_FORBIDDEN_RADIUS].as<double>();
              }
          }
          if(need_update_config_file)
          {
              std::string global_costmap_file = config_file_path_ + "moverglobal.yaml";
              std::ofstream fout(global_costmap_file.c_str());
              if(!fout)
              {
                  feedback[CONFIG_MSG] = "could not open configure file";
                  return false;
              }
              fout << global_costmap_info_;
              fout.close();
          }
      }//global_costmap configure param
      else if(value[CONFIG_PARAM_SPACE].asString() == PARAM_LOCAL_COSTMAP)
      {
          YAML::Node local = local_costmap_info_[PARAM_LOCAL_COSTMAP];

          if(value[CONFIG_PARAM_NAME].asString() == PARAM_UPDATE_FREQ)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify update frequency
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp < 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local[PARAM_UPDATE_FREQ] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local[PARAM_UPDATE_FREQ].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_PUBLISH_FREQ)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify publish frequency
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp < 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local[PARAM_PUBLISH_FREQ] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local[PARAM_PUBLISH_FREQ].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_INFLATION_RADIUS)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify inflation radius
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local[PARAM_INFLATION_RADIUS] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local[PARAM_INFLATION_RADIUS].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_COST_SCALING_FACTOR)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify cost_scaling_factor
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local[PARAM_COST_SCALING_FACTOR] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local[PARAM_COST_SCALING_FACTOR].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_FORBIDDEN_RADIUS)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify forbidden_radius
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local[PARAM_FORBIDDEN_RADIUS] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local[PARAM_FORBIDDEN_RADIUS].as<double>();
              }
          }
          if(need_update_config_file)
          {
              std::string local_costmap_file = config_file_path_ + "moverlocal.yaml";
              std::ofstream fout(local_costmap_file.c_str());
              if(!fout)
              {
                  feedback[CONFIG_MSG] = "could not open configure file";
                  return false;
              }
              fout << local_costmap_info_;
              fout.close();
          }
      }//local costmap configure
      else if(value[CONFIG_PARAM_SPACE].asString() == PARAM_PLANNER)
      {
          if(value[CONFIG_PARAM_NAME].asString() == PARAM_CONTROLLER_FREQ)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify controller frequency
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp < 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      planner_info_[PARAM_CONTROLLER_FREQ] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = planner_info_[PARAM_CONTROLLER_FREQ].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_PLANNER_FREQ)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify planner frequency
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp < 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      planner_info_[PARAM_PLANNER_FREQ] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = planner_info_[PARAM_PLANNER_FREQ].as<double>();
              }
          }
          if(need_update_config_file)
          {
              std::string planner_info_file = config_file_path_ + "moverlocalplanner.yaml";
              std::ofstream fout(planner_info_file.c_str());
              if(!fout)
              {
                  feedback[CONFIG_MSG] = "could not open configure file";
                  return false;
              }
              fout << planner_info_;
              fout.close();
          }
      }//planner move base
      else if(value[CONFIG_PARAM_SPACE].asString() == PARAM_TRAJ_LOCAL_PLANNER)
      {
          YAML::Node local_planner = planner_info_[PARAM_TRAJ_LOCAL_PLANNER];
          if(value[CONFIG_PARAM_NAME].asString() == PARAM_MAX_VEL_X)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify max vel x
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local_planner[PARAM_MAX_VEL_X] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_MAX_VEL_X].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_ACC_LIM_X)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify acc_lim_x
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local_planner[PARAM_ACC_LIM_X] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_ACC_LIM_X].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_ACC_LIM_TH)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify acc_lim_th
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local_planner[PARAM_ACC_LIM_TH] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_ACC_LIM_TH].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_PATH_DIS_BIAS)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify path_distance_bias
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp < 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local_planner[PARAM_PATH_DIS_BIAS] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_PATH_DIS_BIAS].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_GOAL_DIS_BIAS)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify goal_distance_bias
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp < 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local_planner[PARAM_GOAL_DIS_BIAS] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_GOAL_DIS_BIAS].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_OCC_DIS_SCALE)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify occdist_scale
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp < 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local_planner[PARAM_OCC_DIS_SCALE] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_OCC_DIS_SCALE].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_YAW_GOAL_TOL)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify yaw_goal_tolerance
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local_planner[PARAM_YAW_GOAL_TOL] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_YAW_GOAL_TOL].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_XY_GOAL_TOL)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify xy_goal_tolerance
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local_planner[PARAM_XY_GOAL_TOL] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_XY_GOAL_TOL].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_SIM_TIME)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify sim_time
                  tmp = get_double_value(value,CONFIG_PARAM_VALUE);
                  if(tmp <= 0.0)
                  {
                      return false;
                  }
                  else
                  {
                      local_planner[PARAM_SIM_TIME] = tmp;
                      need_update_config_file = true;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_SIM_TIME].as<double>();
              }
          }
          else if(value[CONFIG_PARAM_NAME].asString() == PARAM_LATCH_XY_GOAL)
          {
              if(value[CONFIG_CMD].asString()== PARAM_SET)
              {
                  //modify footprint enable
                  if(value[CONFIG_PARAM_VALUE].isBool())
                  {
                      local_planner[PARAM_LATCH_XY_GOAL] = value[CONFIG_PARAM_VALUE].asBool();
                      need_update_config_file = true;
                  }
                  else
                  {
                      feedback[CONFIG_MSG] = "parm value is wrong";
                      return false;
                  }
              }
              else if(value[CONFIG_CMD].asString() == PARAM_GET)
              {
                  feedback[CONFIG_PARAM_VALUE] = local_planner[PARAM_LATCH_XY_GOAL].as<bool>();
              }
          }
          if(need_update_config_file)
          {
              std::string planner_info_file = config_file_path_ + "moverlocalplanner.yaml";
              std::ofstream fout(planner_info_file.c_str());
              if(!fout)
              {
                  feedback[CONFIG_MSG] = "could not open configure file";
                  return false;
              }
              fout << planner_info_;
              fout.close();
          }
      }
      return true;
  }

  bool MoveBase::configParamService(ConfigParam::Request &req, ConfigParam::Response &resp)
  {
      std::string json_data = req.cmd;
      if(json_data.length() <= 2)
      {
          ROS_ERROR("config data is not right");
          return false;
      }
      Json::Reader reader;
      Json::Value value;
      Json::Value feedback;
      ROS_DEBUG("configure param service,data:%s",json_data.c_str());
      if(reader.parse(json_data,value) && get_config_param_file_)
      {
          if(value.isNull())
          {
              ROS_ERROR("json value is null");
              return false;
          }
          if(value[CONFIG_CMD].isString() && value[CONFIG_PARAM_SPACE].isString()
                  && value[CONFIG_PARAM_NAME].isString())
          {
              ROS_DEBUG("json value is string");
              bool flag = getAndSetConfigParam(value,feedback);
              Json::StyledWriter fast;
              resp.feedback = fast.write(feedback);
              return flag;
          }
          else
          {
              ROS_ERROR("json value1 is null");
              return false;
          }
      }
      else
      {
          ROS_ERROR("json value2 is null");
          return false;
      }
  }

  bool MoveBase::costmapPauseUpdateService(Pause::Request &req, Pause::Response &resp)
  {
      costmap_pause_update_ = req.pause;
      ROS_INFO("move base costmap pause update:%s",costmap_pause_update_ ? "true":"false");
      if(costmap_pause_update_)
      {
          planner_costmap_ros_->customPause();
          controller_costmap_ros_->customPause();
      }
      else
      {
          planner_costmap_ros_->customResume();
          controller_costmap_ros_->customResume();
      }
      resp.error_code = 0;
      return true;
  }

  MoveBase::~MoveBase()
  {
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    fixpath_planner_.reset();
    tc_.reset();
  }

  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL)
    {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose))
    {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);


    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty())
    {
      ROS_DEBUG_NAMED("move_base","Failed to find a  normal plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity()
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q)
  {
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  void MoveBase::registerThread()
  {
      bool registered = false;
      ros::Duration(5.0).sleep();
      ros::Rate r(1);
      while(ros::ok()&&!registered)
      {
          //if(!registered)
          //{
              //call service
              motion_planner::RegisterPlanner srv;
              srv.request.planner = move_base_planner_state;
              if(planner_client.call(srv))
              {
                  ROS_INFO("motion planner register ok!");
                  registered = true;
              }
              else
              {
                  ROS_INFO("motion planner register failed!");
              }
          //}
          r.sleep();
      }
  }
bool MoveBase::check_in_line(std::vector<geometry_msgs::PoseStamped>& path)
  {
		
	  std::vector<robot_info_t> infos;
	  {
		  boost::mutex::scoped_lock lock(robots_pose_mutex_);
		  infos = robots_info_;
	  }
      double dist = 0.0;
      double min_path_dist = 100000;
      std::vector<geometry_msgs::PoseStamped>::iterator path_min;
      for(std::vector<geometry_msgs::PoseStamped>::iterator path_it=path.begin(); path_it != path.end(); path_it++)
      {
          double d = hypot(path_it->pose.position.x - current_pose_.pose.pose.position.x,
                           path_it->pose.position.y - current_pose_.pose.pose.position.y);
          if(d < min_path_dist)
          {
              min_path_dist = d;
              path_min = path_it;
          }
      }

      ROS_INFO("left size:%d,in_line_dist:%f",(int)(path.end()-path_min),in_line_distance_);
      for(std::vector<robot_info_t>::iterator it = infos.begin(); it != infos.end(); it++)
      {
          dist = hypot(it->pose.pose.pose.position.x - current_pose_.pose.pose.position.x,
                       it->pose.pose.pose.position.y - current_pose_.pose.pose.position.y);
          if(dist < in_line_distance_)
          {
              for(std::vector<geometry_msgs::PoseStamped>::iterator path_it=path_min; path_it != path.end(); path_it++)
              {
                  double d = hypot(path_it->pose.position.x - it->pose.pose.pose.position.x,
                                   path_it->pose.position.y - it->pose.pose.pose.position.y);
                  if(d < on_path_distance_)
                  {
                      return true;
                  }
              }
          }
      }
      return false;
  }

//sonar use mode:
//              0: forward mode   forward 6 sonar sensor open
//              1: backward mode  backward 2 sonar sensor open
//              2: rotation mode  all 14 sonar sensor open
//              3: stop mode      all 14 sonar sensor close
void MoveBase::setup_sonar_mode(int mode)
{
    int actual_mode = 999;
    global_node_.param("/noah_sensors/ultrasonic_work_mode_ack", actual_mode, 999);
    if(mode != actual_mode)
    {
        global_node_.setParam("/noah_sensors/ultrasonic_work_mode", mode);
    }
    ROS_DEBUG("actual mode:%d,set mode:%d",actual_mode,mode);
    return;
}

int MoveBase::get_sonar_mode()
{
    int actual_mode = 999;
    global_node_.getParam("/noah_sensors/ultrasonic_work_mode_ack",actual_mode);
    return actual_mode;
}

void MoveBase::check_vel_setup_sonar_mode(geometry_msgs::Twist cmd_vel)
{
    int mode = 999;
    if(cmd_vel.linear.x > 0.01 )
    {
        mode = 0;
    }
    else if(cmd_vel.linear.x < -0.01)
    {
        mode = 1;
    }
    else if(fabs(cmd_vel.angular.z) > 0.001)
    {
        mode = 2;
    }
    if(999 != mode)
    {
        setup_sonar_mode(mode);
    }
    return;
}

void MoveBase::fixpathPlanThread()
{
	ros::NodeHandle n;
    ros::Rate r(20);
	nav_msgs::Path gui_path;
	ros::Time start_time = ros::Time::now();
	geometry_msgs::PoseStamped start;
	geometry_msgs::PoseStamped goal;
	geometry_msgs::Twist cmd_vel;
	geometry_msgs::PoseStamped path_end_point;
    ros::Time last_valid_control_time;
    ros::Time close_sonar_start_time;
    ros::Time wait_costmap_update_start_time;
	boost::unique_lock<boost::mutex> lock(planner_mutex_);
    bool flag = false;
    bool goal_reached_flag = false;
    double sonar_random_stop_time = 1.0;
    double wait_costmap_update_duration_time = 0.0;
    int last_used_sonar_mode;
    unsigned int local_costmap_update_index = 0;

    while(n.ok())
    {
        while(!runFixpathPlanner_)
        {
            setup_sonar_mode(3);
            if(goal_reached_flag)
            {
                Json::Value result;
                result["planner"] = MOVE_FIXPATH_MOTION;
                result["result"] = fixpath_status;
                Json::StyledWriter fast;
                std_msgs::String status;
                status.data = fast.write(result);
                fixpath_motion_status_pub_.publish(status);
                goal_reached_flag = false;
            }
			fixpath_planner_cond_.wait(lock);
		}
		fixpath_plan_state_ = PLANNING;
		start_time = ros::Time::now();

		while(n.ok())
		{
            ROS_INFO("fixpath_status:%d",fixpath_status);
            if(fixpath_status == NAV_READY)
            {
                publishZeroVelocity();
                setup_sonar_mode(3);
                break;
            }
            else if(fixpath_status == NAV_PAUSE)
            {
                publishZeroVelocity();
                setup_sonar_mode(3);
            }
            else
            {
                switch(fixpath_plan_state_)
                {
                    case PLANNING:
                        ROS_DEBUG("fixpath_plan_state_ PLANNING");
                        start.pose.position.x = cur_fixpath_id_;
                        start.header.frame_id = cur_fixpath_scene_name_;
                        goal.pose.position.x = cur_fixpath_id_;
                        goal.header.frame_id = cur_fixpath_map_name_;
                        fixthpath_planner_plan_.clear();
                        flag = fixpath_planner_->makePlan(start, goal ,fixthpath_planner_plan_);
                        if(!flag)
                        {

                            Json::Value result;
                            result["planner"] = MOVE_FIXPATH_MOTION;
			    if(fixthpath_planner_plan_.empty())
			    {
				result["result"] = 11;
				ROS_ERROR("The fixpath can not be found in database!");
			    }
			    else
			    {
				result["result"] = 10;
				ROS_ERROR("Robot far away from the fixpath!");
			    }
                            
                            Json::StyledWriter fast;
                            std_msgs::String status;
                            status.data = fast.write(result);
                            fixpath_motion_status_pub_.publish(status);

                            runFixpathPlanner_ = false;
                            fixpath_status = NAV_READY;
                            
                            break;
                        }

                        gui_path.poses.resize(fixthpath_planner_plan_.size());
                        if (!fixthpath_planner_plan_.empty())
                        {
                            gui_path.header.frame_id = fixthpath_planner_plan_[0].header.frame_id;
                            gui_path.header.stamp = fixthpath_planner_plan_[0].header.stamp;
                        }
                        for (unsigned int i = 0; i < fixthpath_planner_plan_.size(); i++)
                        {
                            gui_path.poses[i] = fixthpath_planner_plan_[i];
                        }

                        path_end_point = fixthpath_planner_plan_[fixthpath_planner_plan_.size() - 1];
                        ROS_DEBUG("path_end_point:[%.3f, %.3f, %.3f]",path_end_point.pose.position.x,path_end_point.pose.position.y,tf::getYaw(path_end_point.pose.orientation));
                        if(!tc_->setPlan(fixthpath_planner_plan_))
                        {
                            ROS_ERROR("Failed to pass global pan to local planner");
                        }
                        else
                        {
                            fixpath_plan_pub_.publish(gui_path);
                            fixpath_plan_state_ = CONTROLLING;
                            last_valid_control_time = ros::Time::now();
                        }
                        publishZeroVelocity();
                        goal_reached_flag = false;
                        break;
                    case CONTROLLING:
                        ROS_DEBUG("fixpath_plan_state_ CONTROLLING");
                        if(ros::Time::now() < wait_costmap_update_start_time + ros::Duration(wait_costmap_update_duration_time))
                        {
                            publishZeroVelocity();
                            break;
                        }
                        if(tc_->isGoalReached())
                        {
                            ROS_DEBUG("fixpath: cur:[ %.3lf, %.3lf, %.3lf], goal:[ %.3lf, %.3lf, %.3lf]",
                                cur_pose.pose.position.x,cur_pose.pose.position.y,cur_pose.pose.position.z,
                                path_end_point.pose.position.x,path_end_point.pose.position.y,path_end_point.pose.position.z);
                            fixpath_status = NAV_READY;
                            runFixpathPlanner_ = false;

                            goal_reached_flag = true;
//                            Json::Value result;
//                            result["planner"] = MOVE_FIXPATH_MOTION;
//                            result["result"] = fixpath_status;
//                            Json::StyledWriter fast;
//                            std_msgs::String status;
//                            status.data = fast.write(result);
//                            fixpath_motion_status_pub_.publish(status);
                            break;
                        }
                        else
                        {
                            bool in_line = check_in_line(fixthpath_planner_plan_);
                            if(in_line)
                            {
								//ROS_INFO("some robot is on the path");
                                publishZeroVelocity();
                                last_valid_control_time = ros::Time::now();
                                {
                                    actionlib_msgs::GoalStatusArray goalstatusarray;
                                    actionlib_msgs::GoalStatus goalstatus;
                                    goalstatusarray.header.stamp = ros::Time::now();
                                    goalstatus.status = 15;
                                    goalstatus.text = "some robot is on the path";
                                    goalstatusarray.status_list.push_back(goalstatus);
                                    motion_status_pub_.publish(goalstatusarray);
                                }
                            }
                            else
                            {
                                cmd_vel.linear.x = 0.0;
                                cmd_vel.linear.y = 0.0;
                                cmd_vel.angular.z = 0.0;
                                boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
								ROS_DEBUG("move base get lock");
                                if(tc_->computeVelocityCommands(cmd_vel))
                                {
                                    ROS_DEBUG("Got vel: %.3lf, %.3lf, %.3lf",cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
                                    last_valid_control_time = ros::Time::now();
                                    vel_pub_.publish(cmd_vel);
                                    check_vel_setup_sonar_mode(cmd_vel);
                                }
                                else
                                {
                                    ROS_DEBUG("Failed to compute vel");
                                    ros::Time attempt_end = last_valid_control_time + ros::Duration(controller_patience_);
                                    vel_pub_.publish(cmd_vel);
                                    //publishZeroVelocity();

                                    //check if we've tried to find a valid control for longer than our time limit
                                    if(ros::Time::now() > attempt_end)
                                    {
                                        fixpath_plan_state_ = CLEARING;
                                        ROS_DEBUG_NAMED("move_base","wait control over time, CLEARING");
                                    }
                                }
								ROS_DEBUG("move base free lock");
                            }
                            ros::Duration(0.02).sleep();
                        }
                        break;
                    case CLEARING:
                        ROS_INFO("fixpath robot stuck CLEARING");
                        if(ros::Time::now() > last_valid_control_time + ros::Duration(controller_patience_*5))
                        {
                            ROS_DEBUG("handle sensor");
                            fixpath_plan_state_ = SENSOR_HANDLE;
                            close_sonar_start_time = ros::Time::now();
                            last_used_sonar_mode = get_sonar_mode();
                            //get a random sonar stop time;
                            sonar_random_stop_time = ros::Time::now().toNSec()%1000;
                            sonar_random_stop_time = sonar_random_stop_time/400;
                        }
                        else
                        {
                            ROS_DEBUG("handle recovery and clear map");
                            fixpath_plan_state_ = CONTROLLING;
                            recovery_behaviors_[0]->runBehavior();
                            wait_costmap_update_start_time = ros::Time::now();
                            local_costmap_update_index = controller_costmap_ros_->update_index_;
                            wait_costmap_update_duration_time = 1.5;
                        }

                        //last_valid_control_time = ros::Time::now();
                        {
                            actionlib_msgs::GoalStatusArray goalstatusarray;
                            actionlib_msgs::GoalStatus goalstatus;
                            goalstatusarray.header.stamp = ros::Time::now();
                            goalstatus.status = 14;
                            goalstatus.text = "fixpath robot stuck in the place,please help me";
                            goalstatusarray.status_list.push_back(goalstatus);
                            motion_status_pub_.publish(goalstatusarray);
                        }
                        publishZeroVelocity();
//                        while(fabs(local_costmap_update_index - controller_costmap_ros_->update_index_) <= 2)
//                        {
//                            ros::Duration(0.1).sleep();
//                        }
                        //ros::Duration(0.5).sleep();
                        break;
                    case SENSOR_HANDLE:
                        if(close_sonar_start_time + ros::Duration(sonar_random_stop_time) > ros::Time::now())
                        {
                            //close all sonar sensor and wait a random time;
                            setup_sonar_mode(3);
                            //ROS_ERROR("handle sensor close all sonar,index:%d",controller_costmap_ros_->update_index_);
                        }
                        else
                        {
                            setup_sonar_mode(last_used_sonar_mode);
                            fixpath_plan_state_ = CONTROLLING;
                            wait_costmap_update_start_time = ros::Time::now();
                            wait_costmap_update_duration_time = 2.0;
                            recovery_behaviors_[0]->runBehavior();
                            ROS_ERROR("handle sensor recovery sonar,index:%d",controller_costmap_ros_->update_index_);
                            //ROS_ERROR("handle sensor after,index:%d",controller_costmap_ros_->update_index_);
                        }
                        publishZeroVelocity();
                        break;
                    default:
                        ROS_ERROR("This case should never be reached, something is wrong, aborting");
                        break;
                }
                if(NAV_READY == fixpath_status)
                {
                    setup_sonar_mode(3);
                    publishZeroVelocity();
                    break;
                }
            }
			r.sleep();
		}
		r.sleep();
	}
    ROS_ERROR("exit fixpath thread!!!");
}

  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    ros::Rate r(50);
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      r.sleep();
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      planner_plan_->clear();
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      actionlib_msgs::GoalStatusArray goalstatusarray;
      actionlib_msgs::GoalStatus goalstatus;
      goalstatusarray.header.stamp = ros::Time::now();
      if(gotPlan)
      {
          goalstatus.status = 11;
          goalstatus.text = "navigation plan ok";
      }
      else
      {
          goalstatus.status = 12;
          goalstatus.text = "navigation plan failed";
      }
      goalstatusarray.status_list.push_back(goalstatus);
      ROS_INFO("move base plan result:%s",goalstatus.text.c_str());
      motion_status_pub_.publish(goalstatusarray);

      if(gotPlan){
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || ++planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
		  ROS_DEBUG("planner over time,change to clearing");
        }
        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    //we have a goal so start the planner
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(controller_frequency_);
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      if(as_->isPreemptRequested()){
        if(as_->isNewGoalAvailable()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          has_recovery_all_place_ = false;
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else {
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      if(done)
        return;

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    tf::Stamped<tf::Pose> global_pose;
    planner_costmap_ros_->getRobotPose(global_pose);
    geometry_msgs::PoseStamped current_position;
    tf::poseStampedTFToMsg(global_pose, current_position);

    //push the feedback out
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index 
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    if(new_global_plan_){
      //make sure to set the new plan flag to false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      if(!tc_->setPlan(*controller_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //the move_base state machine, handles the control logic for navigation
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      case PLANNING:
        {
          boost::mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");
        //check to see if we've reached our goal
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        //check for an oscillation condition
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
        }
        
        {
		//gavin pause
		if(motion_pause_)
		{
		    last_valid_control_ = ros::Time::now();
			publishZeroVelocity();
			break;
		}
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        if(tc_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
			ROS_DEBUG_NAMED("move_base","wait control over time");
          }
          else{
            //otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
		//gavin pause
		if(motion_pause_)
		{
		    last_valid_control_ = ros::Time::now();
			publishZeroVelocity();
			break;
		}
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
        {
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
          if(recovery_index_ ==recovery_behaviors_.size()-1&&recovery_behaviors_.size()!=1)
          {
        	  if(!has_recovery_all_place_)
        	  {
        		  has_recovery_all_place_ = true;
                  //clear all map
        		  recovery_behaviors_[recovery_index_]->runBehavior();
                  wait_all_clear_count_ = 0;
        	  }
			  else
			  {
                  if(wait_all_clear_count_ < wait_all_clear_times_)
                  {
                      //30m range clear
                      recovery_behaviors_[recovery_index_-1]->runBehavior();
                      wait_all_clear_count_++;
                  }
                  else
                  {
                      recovery_behaviors_[recovery_index_]->runBehavior();
                      wait_all_clear_count_ = 0;
                  }
			  }
          }
          else
          {
        	  recovery_behaviors_[recovery_index_]->runBehavior();
          }


          //we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
          if(recovery_index_ > recovery_behaviors_.size() - 1)
          {
              recovery_index_ = recovery_behaviors_.size() - 1;
              ROS_INFO("recovery index stay in clear costmap");
              actionlib_msgs::GoalStatusArray goalstatusarray;
              actionlib_msgs::GoalStatus goalstatus;
              goalstatusarray.header.stamp = ros::Time::now();
              goalstatus.status = 10;
              goalstatus.text = "robot stuck in the place,please help me";
              goalstatusarray.status_list.push_back(goalstatus);
              motion_status_pub_.publish(goalstatusarray);
          }
        }
        else
        {
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

          if(recovery_trigger_ == CONTROLLING_R)
          {
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R)
          {
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R)
          {
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.", 
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.", 
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    ros::NodeHandle private_nh("~");
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
        ros::NodeHandle n("~");
      //n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      //n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);


        {
            //default load first recovery behaviors: in 10m range,clear map
            boost::shared_ptr<nav_core::RecoveryBehavior> extent_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearExtentCostmapRecovery"));
            extent_clear->initialize("10m_extent_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(extent_clear);
        }

        bool enable = false;
        private_nh.getParam("extent_reset_enable",enable);
        if(enable)
        {
        	{
        		boost::shared_ptr<nav_core::RecoveryBehavior> extent_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearExtentCostmapRecovery"));
        		extent_clear->initialize("20m_extent_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        		recovery_behaviors_.push_back(extent_clear);
        	}

        }
        //next, we'll load a recovery behavior to rotate in place
        boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
        if(clearing_rotation_allowed_){
        	rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        	recovery_behaviors_.push_back(rotate);
        }
        //first, we'll load a recovery behavior to clear the costmap
        enable =true;
        private_nh.getParam("conservative_reset_enable",enable);
        if(enable)
        {
        	boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
        	cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        	recovery_behaviors_.push_back(cons_clear);
            
        }

        //default load these two recovery behaviors.
        {
            boost::shared_ptr<nav_core::RecoveryBehavior> extent_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearExtentCostmapRecovery"));
            extent_clear->initialize("30m_extent_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(extent_clear);
        }

        {
            //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(ags_clear);
        }

      
        //we'll rotate in-place one more time
        //if(clearing_rotation_allowed_)
        //    recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }
};
