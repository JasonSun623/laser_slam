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
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#include "lane_follower/lane_following_planner.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(lane_following_planner, LaneFollowingPlanner, , )
PLUGINLIB_EXPORT_CLASS(lane_follower::LaneFollowingPlanner, nav_core::BaseLocalPlanner);
namespace lane_follower {
LaneFollowingPlanner::LaneFollowingPlanner(): tf_(NULL), costmap_ros_(NULL),initialized_(false),rotating_to_goal_(false),reached_goal_(false),backward_mode_(false) {}
LaneFollowingPlanner::~LaneFollowingPlanner()
{
	//delete dsrv_;

	if(check_collision_planner_ != NULL)
		delete check_collision_planner_;

	if(world_model_ != NULL)
		delete world_model_;
}

void LaneFollowingPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
	tf_ = tf;
	costmap_ros_ = costmap_ros;

	reached_goal_ = false;

	ros::NodeHandle node_private("~/" + name);

	//collision_planner_->initialize(name, tf_, costmap_ros_);



	node_private.param("k_ratio_heading_and_dist",k_ratio_,1.5);
	node_private.param("k_path",k_path_,1.0);
	node_private.param("lambda",lambda_,2.0);
	node_private.param("beta",beta_,1.5);
	node_private.param("control_frequency",control_frequency_,10.0);

	node_private.param("xy_goal_tolerance",xy_goal_tolerance_,0.08);
	node_private.param("yaw_goal_tolerance",yaw_goal_tolerance_,0.05);
	//node_private.param("xy_tolerance_latch_",control_frequency_,10.0);
	node_private.param("forward_collision_dist",forward_collision_dist_,0.12);
	node_private.param("backward_collision_dist",backward_collision_dist_,0.20);

	//go no faster than this
	node_private.param("forward_max_vel_lin", forward_max_vel_lin_, 0.25);
	node_private.param("backward_max_vel_lin", backward_max_vel_lin_, 0.25);
	node_private.param("max_vel_th", max_vel_th_, 0.3);

	//minimum velocities to keep from getting stuck
	node_private.param("min_vel_lin", min_vel_lin_, 0.05);
	min_vel_th_ = -1*max_vel_th_;
	node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.1);
	node_private.param("acc_vth", acc_vth_, 0.5);
	node_private.param("acc_vx", acc_vx_, 0.4);
	node_private.param("r_threshold", r_threshold_, 0.55);
	//node_private.param("samples", samples_, 10);
	//if this is true, turn in place to face the new goal instead of arcing toward it
	node_private.param("turn_in_place_first", turn_in_place_first_, false);

	//if turn_in_place_first is true, turn in place if our heading is more than this far from facing the goal location
	//node_private.param("max_heading_diff_before_moving", max_heading_diff_before_moving_, 0.17);

	ros::NodeHandle node;
	initCollisionCheckPlanner();
	odom_sub_ = node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&LaneFollowingPlanner::odomCallback, this, _1));
	//vel_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	initialized_ = true;
	//dsrv_ = new dynamic_reconfigure::Server<lane_follower::LaneFollowingPlannerConfig>(ros::NodeHandle("~/" + name));
	//dynamic_reconfigure::Server<lane_follower::LaneFollowingPlannerConfig>::CallbackType cb = boost::bind(
	//		&LaneFollowingPlanner::reconfigureCB, this, _1, _2);
	//dsrv_->setCallback(cb);
	ROS_DEBUG("Initialized");

	//test_topic = node.subscribe("cmd_vel", 1, &LaneFollowingPlanner::velCallback, this);
}
void LaneFollowingPlanner::reconfigureCB(lane_follower::LaneFollowingPlannerConfig &config, uint32_t level)
{
	boost::mutex::scoped_lock lock(configure_mutux);
	k_ratio_ = config.k_ratio;
	k_path_ = config.k_path;
	beta_ = config.beta;
	lambda_ = config.lambda;

	acc_vx_ = config.acc_vx;
	acc_vth_ = config.acc_vth;
	forward_max_vel_lin_= config.forward_max_vel_lin;
	backward_max_vel_lin_= config.backward_max_vel_lin;
	max_vel_th_= config.max_vel_th;
	min_vel_lin_= config.min_vel_lin;

	//	pre_camera_pose_x_ = config.pre_camera_pose_x;
	//	pre_camera_pose_y_ = config.pre_camera_pose_y;
	//	pre_camera_pose_z_ = config.pre_camera_pose_z;
	//	pre_camera_pose_pitch_ = config.pre_camera_pose_pitch;
	//	back_camera_pose_x_ = config.back_camera_pose_x;
	//	back_camera_pose_y_ = config.back_camera_pose_y;
	//	back_camera_pose_z_ = config.back_camera_pose_z;
	//	back_camera_pose_pitch_ = config.back_camera_pose_pitch;

}
void LaneFollowingPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//we assume that the odometry is published in the frame of the base
	boost::mutex::scoped_lock lock(odom_lock_);
	base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
	base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
	base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
	ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
			base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}
void LaneFollowingPlanner::getRobotVel(tf::Stamped<tf::Pose>& robot_vel) {
	// Set current velocities from odometry
	geometry_msgs::Twist global_vel;
	{
		boost::mutex::scoped_lock lock(odom_lock_);
		global_vel.linear.x = base_odom_.twist.twist.linear.x;
		global_vel.linear.y = base_odom_.twist.twist.linear.y;
		global_vel.angular.z = base_odom_.twist.twist.angular.z;
		robot_vel.frame_id_ = base_odom_.child_frame_id;
	}
	robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
	robot_vel.stamp_ = ros::Time();
}

bool LaneFollowingPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
	//get the current pose of the robot in the odom frame
	tf::Stamped<tf::Pose> robot_pose;
	robot_pose.setIdentity();

	if(global_plan_.empty())
	{
		return false;
	}
	tf::Stamped<tf::Pose> target_pose;
	tf::poseStampedMsgToTF(global_plan_.back(),target_pose);
	double yaw = tf::getYaw(target_pose.getRotation());

	double goal_th = yaw;
	double max_collision_dist;
	if(target_pose.frame_id_ == "rear_frame")
	{
		max_collision_dist = -1*backward_collision_dist_;
		backward_mode_ = true;
	}
	else
	{
		max_collision_dist = forward_collision_dist_;
		backward_mode_ = false;
	}

	boost::mutex::scoped_lock lock(configure_mutux);
	//get current velocity
	tf::Stamped<tf::Pose> robot_vel;
	getRobotVel(robot_vel);
	if (xy_tolerance_latch_ || (getGoalPositionDistance(robot_pose, target_pose) <= xy_goal_tolerance_))
	{

		//if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
		//just rotate in place
		if (latch_xy_goal_tolerance_)
		{
			xy_tolerance_latch_ = true;
		}
		double angle = goal_th;
		//check to see if the goal orientation has been reached
		if (fabs(angle) <= yaw_goal_tolerance_) {
			//set the velocity command to zero
			cmd_vel.linear.x = 0.0;
			cmd_vel.linear.y = 0.0;
			cmd_vel.angular.z = 0.0;
			rotating_to_goal_ = false;
			xy_tolerance_latch_ = false;
			reached_goal_ = true;

		} else {
			rotating_to_goal_ = true;
			if(!rotateToGoal(robot_pose, robot_vel, angle, cmd_vel))
			{
				return false;
			}
		}
		return true;
	}

	//get the curvature between the two poses
	double path_cur = calPathCurvature(robot_pose, target_pose);
	//calculate velocity with path curature
	calVel(path_cur,(target_pose.getOrigin() - robot_pose.getOrigin()).length(),robot_vel,cmd_vel);
	ROS_DEBUG("cur %f,vel %f,%f, %f,%f",path_cur, robot_vel.getOrigin().getX(),tf::getYaw(robot_vel.getRotation()), cmd_vel.linear.x,cmd_vel.angular.z);
	if(!checkCollision(max_collision_dist*2, 0.0,cmd_vel.angular.z))
	{
		cmd_vel.linear.x/=2.0;
	}
	return checkCollision(max_collision_dist, 0.0,cmd_vel.angular.z);
}
bool LaneFollowingPlanner::transformTarget(const tf::TransformListener& tf,const std::string &target_frame, const tf::Stamped<tf::Pose>& target_pose_in_origin_frame,tf::Stamped<tf::Pose>& target_pose )
{
	try
	{
		// get plan_to_global_transform from plan frame to global_frame
		tf.waitForTransform(target_frame, target_pose_in_origin_frame.frame_id_, ros::Time::now(), ros::Duration(0.5));
		tf.transformPose(target_frame,target_pose_in_origin_frame,target_pose);
		return true;
	}
	catch(tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ExtrapolationException& ex) {
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
		return false;
	}
}
void LaneFollowingPlanner::initCollisionCheckPlanner()
{
	world_model_ = new base_local_planner::CostmapModel(*(costmap_ros_->getCostmap()));
	footprint_spec_ = costmap_ros_->getRobotFootprint();

	check_collision_planner_ = new base_local_planner::TrajectoryPlanner(*world_model_, *(costmap_ros_->getCostmap()), footprint_spec_);

}
bool LaneFollowingPlanner::checkCollision(double vx_samp, double vy_samp, double vtheta_samp)
{
	tf::Stamped<tf::Pose> global_pose;
	{
		boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros_->getCostmap()->getMutex()));
		if (!costmap_ros_->getRobotPose(global_pose)) {
			return false;
		}
	}
	//ROS_INFO("POSE:%f,%f,%f",global_pose.getOrigin().x(), global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()));
	std::vector<geometry_msgs::PoseStamped> plan;
	geometry_msgs::PoseStamped pose_msg;
	tf::poseStampedTFToMsg(global_pose, pose_msg);
	plan.push_back(pose_msg);
	check_collision_planner_->updatePlan(plan, true);
	double cost = check_collision_planner_->scoreTrajectory(global_pose.getOrigin().x(), global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()),vx_samp, vy_samp, vtheta_samp,vx_samp, vy_samp, vtheta_samp);
	ROS_DEBUG("cost %f", cost);
	//return true;
	return (cost>=0.0);
}
bool LaneFollowingPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
	if (! isInitialized()) {
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}
	global_plan_.clear();
	global_plan_ = global_plan;
	//when we get a new plan, we also want to clear any latch we may have on goal tolerances
	xy_tolerance_latch_ = false;
	//reset the at goal flag
	reached_goal_ = false;
	return true;
}

double LaneFollowingPlanner::calPathCurvature(const tf::Stamped<tf::Pose>& robot_pose, const tf::Stamped<tf::Pose>& target_pose) const
{
	tf::Vector3 r_vec = target_pose.getOrigin() - robot_pose.getOrigin();
	tf::Vector3 x_vec(1.0,0.0,0.0);
	double r_vec_orientation;
	double s =r_vec.getY()<0.0?-1.0:1.0;
	r_vec_orientation = s* r_vec.angle(x_vec);

	//r_vec_orientation = (r_vec_orientation<=M_PI?r_vec_orientation:M_PI-r_vec_orientation);
	double robot_orientation = tf::getYaw(robot_pose.getRotation());
	double target_orientation = tf::getYaw(target_pose.getRotation());
	double r_length = r_vec.length();
	//double delta = r_vec.angle(x_vec)*target_pose.getOrigin().
	double theta = angles::normalize_angle(target_orientation - r_vec_orientation);
	double delta = angles::normalize_angle(robot_orientation - r_vec_orientation);
	//ROS_INFO("current robot pose %f,%f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y());
	//ROS_INFO("target pose %f,%f,%f", target_pose.getOrigin().x(), target_pose.getOrigin().y(), target_orientation);
	//ROS_INFO("r_vec_orientation: %f, delta:%f,theta:%f ",r_vec_orientation,delta,theta);

	double arctan = std::atan(-1*k_ratio_*theta);
	ROS_INFO("r:%f, arctan: %f",r_length, arctan);
	double adjust_k_ratio = backward_mode_?k_ratio_*0.2:k_ratio_;
	return (-1/r_length*(k_path_*(delta-arctan)+(1+adjust_k_ratio/(1+adjust_k_ratio*adjust_k_ratio*theta*theta))*std::sin(delta)));
}

void LaneFollowingPlanner::calVel(double cur,double r, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel) const
{
	double max_vel_x ;//= max_vel_lin_;
	double min_vel_x = min_vel_lin_;
	double max_vel_theta = max_vel_th_;
	double min_vel_theta = min_vel_th_;
	double acc_x = acc_vx_;
	double acc_theta = acc_vth_;
	double vx = robot_vel.getOrigin().getX();
	double vth = tf::getYaw(robot_vel.getRotation());
	//vx = backward_mode_?-1*vx:vx;
	if(backward_mode_)
	{
		vx *=-1.0;
		max_vel_x = backward_max_vel_lin_;
	}
	else
	{
		max_vel_x = forward_max_vel_lin_;
	}
	//vth = backward_mode_?-1*vth:vth;
	double vel_x = (max_vel_x/(1+beta_*std::pow(std::fabs(cur),lambda_)));
	double vel_th = vel_x *cur;
	ROS_DEBUG("cal vel %f",vel_x);
	vel_x = (std::min(vel_x,std::max(min_vel_x,max_vel_x*r/r_threshold_)));
	ROS_DEBUG("vel %f  %f %f",vel_x,min_vel_x,max_vel_x*r/r_threshold_);
	if(vel_x>vx)
	{
		vel_x = (std::min(vel_x, vx + acc_x / control_frequency_));

	}
	else
	{
		vel_x = (std::max(vel_x, vx - acc_x / control_frequency_));
	}
	//if(std::fabs(vel_x - vx) < acc_x / control_frequency_)
	//vel_x = vx;


	//collision_planner_.checkTrajectory(vel_x,0.0,vel_th,true);
	/*double nearest_vx = max_vel_x,nearest_vth,step =1/samples_;
	for(int i = 0; i<1; i+=step)
	{	
		if(std::fabs(vx-max_vel_x*i) < nearest_vx)
		{
           nearest_vx = max_vel_x*i
		}

	}
	 */
	if(cur>0)
	{
		vel_th = std::min(vel_th, vth + acc_theta / control_frequency_);
	}
	else
	{
		vel_th = std::max(vel_th, vth - acc_theta / control_frequency_);
	}
	if(std::fabs(cur)>2.0)
	{
		vel_th = 0.1*vel_th/(std::fabs(vel_th)+0.0001);
	}
	cmd_vel.angular.z = vel_th;
	cmd_vel.linear.x = vel_x;


}



bool LaneFollowingPlanner::isGoalReached()
{
	return reached_goal_;
}
double LaneFollowingPlanner::getGoalPositionDistance(const tf::Stamped<tf::Pose>& robot_pose, const tf::Stamped<tf::Pose>& target_pose)
{
	return robot_pose.getOrigin().distance(target_pose.getOrigin());
}

bool LaneFollowingPlanner::rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel){
	double yaw = tf::getYaw(global_pose.getRotation());
	double vel_yaw = tf::getYaw(robot_vel.getRotation());
	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	double ang_diff = goal_th;//angles::shortest_angular_distance(yaw, goal_th);

	double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
			std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_,
					std::min(-1.0 * min_in_place_vel_th_, ang_diff));
	//ROS_INFO("v_theta_samp1:%f, %f,%f,%f,%f",ang_diff,max_vel_th_,min_vel_th_,min_in_place_vel_th_,v_theta_samp);
	//take the acceleration limits of the robot into account
	double max_acc_vel = fabs(vel_yaw) + acc_vth_ / control_frequency_;
	double min_acc_vel = fabs(vel_yaw) - acc_vth_ / control_frequency_;

	v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);
	//ROS_INFO("v_theta_samp2:%f",v_theta_samp);
	//we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
	double max_speed_to_stop = sqrt(2 * acc_vth_ * fabs(ang_diff));

	v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));
	//ROS_INFO("v_theta_samp3:%f",v_theta_samp);
	// Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
	v_theta_samp = v_theta_samp > 0.0
			? std::min( max_vel_th_, std::max( min_in_place_vel_th_, v_theta_samp ))
	: std::max( min_vel_th_, std::min( -1.0 * min_in_place_vel_th_, v_theta_samp ));
	//ROS_INFO("v_theta_samp4:%f",v_theta_samp);
	//we still want to lay down the footprint of the robot and check if the action is legal

	bool valid_cmd = checkCollision(cmd_vel.linear.x, cmd_vel.linear.y, v_theta_samp);

	ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

	if(valid_cmd){
		cmd_vel.angular.z = v_theta_samp;
		return true;
	}

	cmd_vel.angular.z = 0.0;
	return false;

}

};
