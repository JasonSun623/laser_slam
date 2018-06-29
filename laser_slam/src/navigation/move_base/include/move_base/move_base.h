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
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"
#include "move_base/Pause.h"
#include "move_base/ConfigParam.h"
#include "move_base/FixpathNav.h"
#include "motion_planner/motion_planner_state.h"
#include "json/json.h"
#include "yaml-cpp/yaml.h"
#include "fixpath_global_planner/fixpath_global_planner.h"
#include "nav_msgs/Odometry.h"
#include "motion_planner/MotionOp.h"

#define CONFIG_CMD           ("cmd")
#define CONFIG_PARAM_SPACE   ("param_space")
#define CONFIG_PARAM_NAME    ("param_name")
#define CONFIG_PARAM_VALUE   ("param_value")
#define CONFIG_MSG           ("param_msg")

#define PARAM_SET                 ("param_set")
#define PARAM_GET                 ("param_get")
#define COM_COSTMAP               ("com_costmap")
#define PARAM_RAYTACE_RANGE       ("raytrace_range")
#define PARAM_OBSTACLE_RANGE      ("obstacle_range")
#define PARAM_ROBOT_RADIUS        ("robot_radius")
#define PARAM_FOOTPRINT_ENABLE    ("footprint_enable")
#define PARAM_GLOBAL_COSTMAP      ("global_costmap")
#define PARAM_LOCAL_COSTMAP       ("local_costmap")
#define PARAM_UPDATE_FREQ         ("update_frequency")
#define PARAM_PUBLISH_FREQ        ("publish_frequency")
#define PARAM_INFLATION_RADIUS    ("inflation_radius")
#define PARAM_COST_SCALING_FACTOR ("cost_scaling_factor")
#define PARAM_FORBIDDEN_RADIUS    ("forbidden_radius")
#define PARAM_PLANNER             ("planner")
#define PARAM_TRAJ_LOCAL_PLANNER  ("TrajectoryPlannerROS")
#define PARAM_CONTROLLER_FREQ     ("controller_frequency")
#define PARAM_PLANNER_FREQ        ("planner_frequency")
#define PARAM_MAX_VEL_X           ("max_vel_x")
#define PARAM_ACC_LIM_X           ("acc_lim_x")
#define PARAM_ACC_LIM_TH          ("acc_lim_th")
#define PARAM_PATH_DIS_BIAS       ("path_distance_bias")
#define PARAM_GOAL_DIS_BIAS       ("goal_distance_bias")
#define PARAM_OCC_DIS_SCALE       ("occdist_scale")
#define PARAM_YAW_GOAL_TOL        ("yaw_goal_tolerance")
#define PARAM_XY_GOAL_TOL         ("xy_goal_tolerance")
#define PARAM_SIM_TIME            ("sim_time")
#define PARAM_LATCH_XY_GOAL       ("latch_xy_goal_tolerance")

#define MSG_PUB_NAME     ("pub_name")
#define MSG_SUB_NAME     ("sub_name")
#define MSG_ERROR_CODE   ("error_code")
#define MSG_MSG          ("msg")
#define MSG_VALUE        ("value")
#define MSG_DATA         ("data")

#define CMD_FIXPATH_MOTION_START     ("fixpath_motion_start")
#define CMD_FIXPATH_MOTION_CANCEL    ("fixpath_motion_cancel")
#define CMD_FIXPATH_MOTION_PAUSE     ("fixpath_motion_pause")
#define CMD_FIXPATH_MOTION_RESUME    ("fixpath_motion_resume")

#define MOVE_FIXPATH_MOTION             ("fixpath_motion")

#define ROBOTS_CURRENT_POSE             ("robots_current_pose")


namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum MoveBaseState {
    PLANNING,
    CONTROLLING,
    CLEARING,
    SENSOR_HANDLE
  };

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

typedef enum{
    NAV_READY = 0,
    NAV_MOVING = 1,
    NAV_PAUSE = 2,
}nav_status_e;

typedef enum GlobalPlannerType
{
	NORMAL_GLOBAL_PLANNER,
	FIXPATH_GLOBAL_PLANNER,
}global_planner_t;

typedef struct{
    std::string robot_id;
    std::string scene_name;
    std::string map_name;
    nav_msgs::Odometry pose;
}robot_info_t;

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @param global_plan A reference to the global plan being used
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      //gavin add two service to pause motion and costmap update.
      bool pauseService(Pause::Request &req, Pause::Response &resp);
      bool configParamService(ConfigParam::Request &req, ConfigParam::Response &resp);
      bool costmapPauseUpdateService(Pause::Request &req, Pause::Response &resp);
      bool getAndSetConfigParam(Json::Value value,Json::Value &feedback);
      double get_double_value(Json::Value value,std::string item);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();
      void fixpathPlanThread();

      Json::Value HandleCmd(Json::Value cmd);

      void registerThread();//gavin add to register move base

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);
      bool fixpath_nav_service(motion_planner::MotionOp::Request &req,motion_planner::MotionOp::Response &res);

	  void current_pose_callback(const nav_msgs::Odometry& msg);
      void robots_info_callback(const std_msgs::String& msg);
      bool check_in_line(std::vector<geometry_msgs::PoseStamped>& path);
      void setup_sonar_mode(int mode);
      int get_sonar_mode();
      void check_vel_setup_sonar_mode(geometry_msgs::Twist cmd_vel);

      tf::TransformListener& tf_;

      MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      boost::shared_ptr<nav_core::BaseGlobalPlanner> fixpath_planner_;
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      unsigned int recovery_index_;
      int32_t wait_all_clear_count_,wait_all_clear_times_;

      tf::Stamped<tf::Pose> global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, motion_status_pub_,fixpath_motion_status_pub_;
      ros::Subscriber goal_sub_,robots_info_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      ros::ServiceServer motion_control_srv_, costmap_pause_update_srv_, config_param_srv_;
      ros::ServiceClient planner_client;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool motion_pause_,costmap_pause_update_,get_config_param_file_;
      double oscillation_timeout_, oscillation_distance_, in_line_distance_, on_path_distance_;
      motion_planner::motion_planner_state move_base_planner_state;
      std::string config_file_path_;
      YAML::Node com_costmap_info_, global_costmap_info_, local_costmap_info_, planner_info_;
      ros::Publisher fixpath_plan_pub_;
      ros::NodeHandle global_node_;

      MoveBaseState state_;
      MoveBaseState fixpath_plan_state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> fgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      //for fixpath plan
      std::vector<geometry_msgs::PoseStamped>  fixthpath_planner_plan_;

      //set up the planner's thread
      bool runPlanner_;
      bool runFixpathPlanner_;
      boost::mutex planner_mutex_, robots_pose_mutex_;
      boost::condition_variable planner_cond_;
      boost::condition_variable fixpath_planner_cond_;
      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;
      boost::thread* register_thread_;
      boost::thread* fixpath_planner_thread_;
      unsigned int cur_fixpath_id_;
      unsigned int fixpath_goal_tolerance_type_;
      std::string cur_fixpath_scene_name_,cur_fixpath_map_name_;
	  geometry_msgs::PoseStamped cur_pose;
      nav_msgs::Odometry current_pose_;
	  ros::Subscriber current_pose_sub;

      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
      bool has_recovery_all_place_;
      ros::ServiceServer fixpath_nav_srv;
      bool fixpath_goal_reached_;
      nav_status_e fixpath_status;
      std::vector<robot_info_t> robots_info_;
	  std::string self_id_;
  };
};
#endif

