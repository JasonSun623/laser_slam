/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
* Author: Tom Zhang
*********************************************************************/
#ifndef LANE_FOLLOWING_PLANNER_LANE_FOLLOWING_PLANNER_H_
#define LANE_FOLLOWING_PLANNER_LANE_FOLLOWING_PLANNER_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <lane_follower/LaneFollowingPlannerConfig.h>
namespace lane_follower {
  class LaneFollowingPlanner : public nav_core::BaseLocalPlanner {
    public:
      LaneFollowingPlanner();
      ~LaneFollowingPlanner();
      void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool isGoalReached();
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool checkCollision(double vx_samp, double vy_samp, double vtheta_samp);
    private:
      inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
      }
      bool isInitialized() {
        return initialized_;
      }
      void reconfigureCB(lane_follower::LaneFollowingPlannerConfig &config, uint32_t level);
      bool getTargetPoseFromCamera(tf::Stamped<tf::Pose>& target_pose);
      void getRobotVel(tf::Stamped<tf::Pose>& robot_vel);
      void calVel(double cur,double r, const tf::Stamped<tf::Pose>& robot_vel,geometry_msgs::Twist& cmd_vel) const;

      double calPathCurvature(const tf::Stamped<tf::Pose>& robot_pose, const tf::Stamped<tf::Pose>& target_pose) const;
      bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
          const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);
      bool transformTarget(const tf::TransformListener& tf,
    		  const std::string &target_frame,
			  const tf::Stamped<tf::Pose>&target_pose_in_origin_frame,
			  tf::Stamped<tf::Pose>& target_pose);
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      void initCollisionCheckPlanner();
      //void velCallback (const geometry_msgs::Twist& cmd_vel);
      double getGoalPositionDistance(const tf::Stamped<tf::Pose>& robot_pose, const tf::Stamped<tf::Pose>& target_pose);
      bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);
      double getGoalOrientationAngleDifference(const tf::Stamped<tf::Pose>& global_pose, double goal_th) {
        double yaw = tf::getYaw(global_pose.getRotation());
        return angles::shortest_angular_distance(yaw, goal_th);
      }
      tf::TransformListener* tf_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      base_local_planner::WorldModel* world_model_;
      base_local_planner::TrajectoryPlanner* check_collision_planner_;
      std::vector<geometry_msgs::Point> footprint_spec_;
      //ros::Publisher vel_pub_;
      bool initialized_;
      double backward_max_vel_lin_,forward_max_vel_lin_,max_vel_th_;
      double min_vel_lin_, min_vel_th_;
      double acc_vx_,acc_vth_,min_in_place_vel_th_;
      double control_frequency_;
      double r_threshold_;

      bool turn_in_place_first_,backward_mode_;
      double max_heading_diff_before_moving_;


      double forward_collision_dist_,backward_collision_dist_;
      double pre_camera_pose_x_,pre_camera_pose_y_,pre_camera_pose_z_,pre_camera_pose_pitch_;
      double back_camera_pose_x_,back_camera_pose_y_,back_camera_pose_z_,back_camera_pose_pitch_;
      double k_ratio_,k_path_, lambda_, beta_;

      boost::mutex odom_lock_, configure_mutux;
      ros::Subscriber odom_sub_;
      nav_msgs::Odometry base_odom_;

      std::vector<geometry_msgs::PoseStamped> global_plan_;

      tf::Stamped<tf::Pose> last_traget_pose_;

      double xy_goal_tolerance_,yaw_goal_tolerance_;
      bool xy_tolerance_latch_,latch_xy_goal_tolerance_,reached_goal_,rotating_to_goal_;
      dynamic_reconfigure::Server<lane_follower::LaneFollowingPlannerConfig> *dsrv_;

  };
};
#endif
