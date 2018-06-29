/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Christian Connette
 *********************************************************************/

#ifndef EBAND_LOCAL_PLANNER_ROS_H_
#define EBAND_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// classes wich are parts of this pkg
#include <mrobot_local_planner/mrobot_local_planner.h>
#include <mrobot_local_planner/conversions_and_types.h>
#include <mrobot_local_planner/mrobot_visualization.h>
#include <mrobot_local_planner/mrobot_trajectory_controller.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>
#include <motion_planner/MotionOp.h>

// msgs
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <costmap_2d/costmap_2d.h>
#include <bits/stl_algobase.h>

#include "json/json.h"

typedef enum{
    DISTANCE_METHOD = 0,
    TIME_METHOD
}collision_method_e;

namespace mrobot_local_planner{

  /**
   * @class EBandPlannerROS
   * @brief Plugin to the ros base_local_planner. Implements a wrapper for the Elastic Band Method
   */
  class MrobotPlannerROS : public nav_core::BaseLocalPlanner{

    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      MrobotPlannerROS();

      /**
       * @brief Constructs the ros wrapper
       * @param name The name to give this instance of the elastic band local planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      MrobotPlannerROS(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~MrobotPlannerROS();

      /**
       * @brief Initializes the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Set the plan that the controller is following; also reset eband-planner
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

    private:

      // pointer to external objects (do NOT delete object)
      costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap
      costmap_2d::Costmap2D* costmap_;
      tf::TransformListener* tf_; ///<@brief pointer to Transform Listener

      // parameters
      double yaw_goal_tolerance_, xy_goal_tolerance_; ///<@brief parameters to define region in which goal is treated as reached
      double rot_stopped_vel_, trans_stopped_vel_; ///<@brief lower bound for absolute value of velocity (with respect to stick-slip behaviour)

      // Topics & Services
      ros::Publisher g_plan_pub_; ///<@brief publishes modified global plan
      ros::Publisher l_plan_pub_; ///<@brief publishes prediction for local commands
      ros::Subscriber odom_sub_; ///<@brief subscribes to the odometry topic in global namespace
      ros::ServiceServer param_update_srv_;

      // data
      nav_msgs::Odometry base_odom_;
      std::vector<geometry_msgs::PoseStamped> global_plan_; // plan as handed over from move_base or global planner
      std::vector<geometry_msgs::PoseStamped> transformed_plan_; // plan transformed into the map frame we are working in
      std::vector<int> plan_start_end_counter_; // stores which number start and end frame of the transformed plan have inside the global plan

      // pointer to locally created objects (delete - except for smart-ptrs:)
      boost::shared_ptr<MrobotPlanner> eband_;
      boost::shared_ptr<MrobotVisualization> eband_visual_;
      boost::shared_ptr<MrobotTrajectoryCtrl> eband_trj_ctrl_;

      base_local_planner::CostmapModel* world_model_;
      std::vector<geometry_msgs::Point> footprint_spec_;

      double inscribed_radius_, circumscribed_radius_;

      tf::TransformListener tf_listener;

      bool goal_reached_;
      bool xy_tolerance_latch_;
      std::string global_frame_;

      double max_vx_;
      double max_vy_;
      double max_vth_;
      double min_vth_;
      double actual_max_vx_;
      double actual_max_vy_;
      double actual_max_vth_;
      double max_acc_x_;
      double max_acc_y_;
      double max_acc_th_;
      double look_head_dist_;
      double max_obstacle_dist_;
      double max_obstacle_time_;
      double obstacle_detect_center_;
      double obstacle_detect_max_radius_;
      double curvature_dist_;
      double max_rotate_adjust_th_;
      double control_freq_;
      double dth_acc_;
      double min_stop_dist_;
      double update_max_vel_x_;
      double update_acc_x_;
      double update_max_vel_th_;
      double update_acc_th_;
      double update_min_stop_dist_;
      bool update_flag_;
      ros::Time last_stop_time;

      // flags
      bool initialized_;
      boost::mutex odom_mutex_; // mutex to lock odometry-callback while data is read from topic


      // methods

      /**
       * @brief Odometry-Callback: function is called whenever a new odometry msg is published on that topic 
       * @param Pointer to the received message
       */
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      bool calculateVel(geometry_msgs::Twist& cmd_vel,const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Odometry &base_odom);
      bool modify_path(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan,std::vector<geometry_msgs::PoseStamped>& global_plan);
      bool transformGlobalPlan(const tf::TransformListener& tf,const std::vector<geometry_msgs::PoseStamped>& global_plan,
          const tf::Stamped<tf::Pose>& global_pose,const costmap_2d::Costmap2D& costmap,const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);
      double adjust_th(double th);
      bool checkCollision(double& vx,double& vy,double& th,collision_method_e method);
      void stopWithAcc(double& vx,double& vy,double& vth);
      void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
      bool calculateActualMaxVel(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan,double& path_curvature);
      bool calculateMaxVelScale(const tf::Stamped<tf::Pose>& global_pose,double& vx_scale,double& vy_scale,double& vth_scale);
      bool updateParams(motion_planner::MotionOp::Request &req, motion_planner::MotionOp::Response &res);
  };
};
#endif


