/*
 * lane_follower.cpp
 *
 *  Created on: Jul 12, 2017
 *      Author: tomzhang
 */
#include "lane_follower/lane_follower.h"
#include <boost/algorithm/string.hpp>
#include <std_srvs/Empty.h>
using costmap_2d::FREE_SPACE;
namespace lane_follower{

LaneFollower::LaneFollower(tf::TransformListener &tf):tf_(tf), controller_costmap_ros_(NULL),
		blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),recevied_ms_(STOP),sub_ms(MOVE),last_ms_(STOP),execute_ms_(FORWARD),stop_thread_(true),
		has_check_right_(false),find_rear_target_(false),find_front_target_(false),max_rotation_repeat_(1),current_rotation_repeat_(0),
		has_check_left_(false),init_line_pose_(false),initialized_(false)
{
	ros::NodeHandle private_nh("~");
	ros::NodeHandle nh;
	service_ = private_nh.advertiseService("missions",&LaneFollower::missionService,this);

	start_front_camera_client_= nh.serviceClient<std_srvs::Empty>("/usb_cam_0/usb_cam/start_capture");
	start_rear_camera_client_ = nh.serviceClient<std_srvs::Empty>("/usb_cam_1/usb_cam/start_capture");
	stop_front_camera_client_ = nh.serviceClient<std_srvs::Empty>("/usb_cam_0/usb_cam/stop_capture");
	stop_rear_camera_client_  = nh.serviceClient<std_srvs::Empty>("/usb_cam_1/usb_cam/stop_capture");
	vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
	odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&LaneFollower::odomCallback, this, _1));
	lane_detector_pub_ = private_nh.advertise<std_msgs::String>("camera_using_n",1);
	motion_status_pub_ = nh.advertise<std_msgs::String>("motion_status",1);
	lane_detector_sub_ = nh.subscribe("camera",10,&LaneFollower::receiveLineCallback, this);
	std::string  control_planner;
	private_nh.param("control_planner", control_planner, std::string("lane_follower/LaneFollowingPlanner"));
	//private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));

	//we'll assume the radius of the robot to be consistent with what's specified for the costmaps
	private_nh.param("controller_costmap/inscribed_radius", inscribed_radius_, 0.26);
	private_nh.param("controller_costmap/circumscribed_radius", circumscribed_radius_, 0.60);
	private_nh.param("controller_frequency", controller_frequency_, 10.0);
	private_nh.param("max_wait_time", controller_patience_, 20.0);

	private_nh.param("pre_camera_pose_x",pre_camera_pose_x_,0.59);
	private_nh.param("pre_camera_pose_y",pre_camera_pose_y_,0.0);

	private_nh.param("back_camera_pose_x",back_camera_pose_x_,0.54);
	private_nh.param("back_camera_pose_y",back_camera_pose_y_,0.0);
	private_nh.param("max_stop_dist", max_stop_dist_,0.25);
	private_nh.param("max_turn_left",max_turn_left_angle_,0.5);
	private_nh.param("max_turn_right",max_turn_right_angle_,0.65);
	private_nh.param("max_rotation_repeat",max_rotation_repeat_,2);
	private_nh.param("image_width",image_width_,160);
	private_nh.param("image_height",image_height_,120);
	private_nh.param("gradient",gradient_,M_PI);
	private_nh.param("min_global_goal_dist",min_global_goal_dist_,0.2);
	private_nh.param("camera_goal_toloranz",camera_goal_toloranz_,0.17);
	private_nh.param("using_amcl",using_amcl_,false);
	private_nh.param("front_pixel_heigth",front_pixel_heigth_,0.0005);
	private_nh.param("front_pixel_width",front_pixel_width_,0.002);
	private_nh.param("back_pixel_heigth",back_pixel_heigth_,0.001);
	private_nh.param("back_pixel_width",back_pixel_width_,0.002);
	private_nh.param("find_lane_waiting_time",find_lane_waiting_time_,120.0);
	//find_lane_waiting_time_ *=60.0;
	//create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
	controller_costmap_ros_ = new costmap_2d::Costmap2DROS("controller_costmap", tf_);
	controller_costmap_ros_->pause();
	try
	{
		//check if a non fully qualified name has potentially been passed in
		if(!blp_loader_.isClassAvailable(control_planner))
		{
			std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
			for(unsigned int i = 0; i < classes.size(); ++i)
			{
				if(control_planner == blp_loader_.getName(classes[i]))
				{
					//if we've found a match... we'll get the fully qualified name and break out of the loop
					ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
							control_planner.c_str(), classes[i].c_str());
					control_planner = classes[i];
					break;
				}
			}
		}

		control_planner_ = blp_loader_.createInstance(control_planner);
		control_planner_->initialize(blp_loader_.getName(control_planner),&tf_, controller_costmap_ros_);
	} catch (const pluginlib::PluginlibException& ex)
	{
		ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", control_planner.c_str(), ex.what());
		exit(1);
	}
	controller_costmap_ros_->start();
	std::vector<std::string> clearable_layers_default, clearable_layers;
	clearable_layers_default.push_back( std::string("obstacle_layer") );
	private_nh.param("layer_names", clearable_layers, clearable_layers_default);

	for(unsigned i=0; i < clearable_layers.size(); i++) {
		ROS_INFO("Lane Follower will clear layer %s", clearable_layers[i].c_str());
		clearable_layers_.insert(clearable_layers[i]);
	}
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response res;
	while(!stop_front_camera_client_.call(req,res))
	{
		ROS_ERROR("wait for shut down front camera!");
		ros::Rate(2).sleep();
	}
	while(!stop_rear_camera_client_.call(req,res))
	{
		ROS_ERROR("wait for shut down rear camera!");
		ros::Rate(2).sleep();
	}
	mission_thread_ = boost::thread(boost::bind(&LaneFollower::executeMS,this));
	mission_thread_.detach();
	initialized_ = true;

}
LaneFollower::~LaneFollower()
{

	if(controller_costmap_ros_ != NULL)
		delete controller_costmap_ros_;
	control_planner_.reset();

}
//void LaneFollower::reconfigureCB(LaneFollower::MoveBaseConfig &config, uint32_t level)
///{/
//    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

//}
void LaneFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//we assume that the odometry is published in the frame of the base
	boost::mutex::scoped_lock lock(odom_lock_);
	base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
	base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
	base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
	base_odom_.pose.pose= msg->pose.pose;
	ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
			base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}
void LaneFollower::receiveLineCallback(const std_msgs::String::ConstPtr &msg)
{

	std::vector<std::string> vecSegTag;
	std::string str(msg->data.c_str());
	//ROS_INFO("%s",str.c_str());
	boost::split(vecSegTag, str,boost::is_any_of("/"));
	//ROS_INFO("size:%d",(int)vecSegTag.size());
//	if (vecSegTag.size() != 8)
//	{
//		return;
//	}
	int camera_no = atoi(vecSegTag[0].c_str());


	if(camera_no ==0)
	{
		int isValid = atoi(vecSegTag[1].c_str());
		if (isValid != 1)
		{
			return;
		}
		int isFoundGoal = atoi(vecSegTag[2].c_str());

		if (vecSegTag[3]=="None")
		{
			return;
		}
		double angle = atof(vecSegTag[3].c_str());
		int idx_width =  atoi(vecSegTag[4].c_str());
		int idx_height =  atoi(vecSegTag[5].c_str());
		double x = (image_height_- idx_height)*front_pixel_heigth_ + 0.69;
		double y = (fabs(angle)>M_PI*gradient_?0.0:(image_width_/2- idx_width)*front_pixel_width_);
		//angle = angleClassfication(angle);

		double last_angle = tf::getYaw(last_line_pose_.getRotation());
		angle = std::fabs(last_angle - angle)>0.05?angle:last_angle;


		boost::mutex::scoped_lock lock(goal_mtx);
		if(isFoundGoal == 1)
		{

			recordCurrentPose(last_robot_pose_);
			if(vecSegTag.size() == 9)
				dist_to_final_goal_from_camera_ = std::min(0.5,std::max(0.0, atof(vecSegTag[8].c_str())));
			else
				dist_to_final_goal_from_camera_ = 0.15;
			find_front_target_ = isFoundGoal;
		}
		new_front_pose_.frame_id_ = "front_frame";
		new_front_pose_.setIdentity();
		new_front_pose_.getOrigin().setX(x);
		new_front_pose_.getOrigin().setY(y);
		new_front_pose_.getOrigin().setZ(0.0);
		new_front_pose_.setRotation(tf::createQuaternionFromYaw(angle));
		new_front_pose_.stamp_ = ros::Time::now();
		//ROS_INFO("set front goal %f, %f, %f", new_front_pose_.getOrigin().getX(), new_front_pose_.getOrigin().getY(), tf::getYaw(new_front_pose_.getRotation()));
		this->received_front_line_ = true;

	}
	else if(camera_no ==1)
	{
		int isValid = atoi(vecSegTag[1].c_str());
		if (isValid != 1)
		{
			return;
		}
		int isFoundGoal = atoi(vecSegTag[2].c_str());
		if (vecSegTag[3]=="None")
		{
			return;
		}
		double angle = atof(vecSegTag[3].c_str());
		int idx_width =  atoi(vecSegTag[4].c_str());
		int idx_height =  atoi(vecSegTag[5].c_str());
		double x = (image_height_- idx_height)*back_pixel_heigth_ + 0.65;
		double y = (fabs(angle)>M_PI*gradient_?0.0:(image_width_/2- idx_width)*back_pixel_width_);
		//angle = angleClassfication(angle);
		double last_angle = tf::getYaw(last_line_pose_.getRotation());
		angle = std::fabs(last_angle - angle)>0.05?angle:last_angle;

		boost::mutex::scoped_lock lock(goal_mtx);
		if(isFoundGoal == 1)
		{
			recordCurrentPose(last_robot_pose_);
			if(vecSegTag.size() == 9)
				dist_to_final_goal_from_camera_ = std::min(0.5,std::max(0.0, atof(vecSegTag[8].c_str())));
			else
				dist_to_final_goal_from_camera_ = 0.15;
			find_rear_target_ = isFoundGoal;

		}
		new_rear_pose_.frame_id_ = "rear_frame";
		new_rear_pose_.setIdentity();
		new_rear_pose_.getOrigin().setX(x);
		new_rear_pose_.getOrigin().setY(y);
		new_rear_pose_.getOrigin().setZ(0.0);
		new_rear_pose_.setRotation(tf::createQuaternionFromYaw(angle));
		new_rear_pose_.stamp_ = ros::Time::now();
		//ROS_INFO("set new rear goal %f, %f, %f", new_rear_pose_.getOrigin().getX(), new_rear_pose_.getOrigin().getY(), tf::getYaw(new_rear_pose_.getRotation()));
		this->received_rear_line_ = true;
	}

}
bool LaneFollower::missionService(motion_planner::MotionOp::Request  &req,
		motion_planner::MotionOp::Response &res)
{
	boost::mutex::scoped_lock lock(mission_mtx_);

	std::string test =req.cmd;
	ROS_INFO("%s",test.c_str());
	Json::Reader reader;
	Json::Value value;
	Json::Value result;
	if(reader.parse(test,value))
	{
		if(value[MSG_PUB_NAME].isNull())
		{
			ROS_INFO("pub_name value is empty");
		}
		else
		{
			if(!initialized_)
			{
				res.result = "lane follow did not init";
				res.error_code = 2;
				return true;
			}
			has_new_mission_ =true;
			//if it is not empty.
			result = handleCmd(value);
			Json::StyledWriter fast;
			res.result = fast.write(result);
			res.error_code = 0;
			return true;
		}
	}
	res.result = "op failed!";
	res.error_code = 1;
	return true;
}
void LaneFollower::executeMS()
{
	bool publish;
	tf::Stamped<tf::Pose> line_pose;
	geometry_msgs::Twist cmd_vel;

	std_msgs::String motion_status;
	Json::Value motion_status_json;
	motion_status_json["planner"] = MOVE_LANE_FOLLOWING_MOTION;
	EMission recevied_ms;


	ros::Rate r(controller_frequency_);
	while(ros::ok())
	{
		//ROS_INFO("lane follower thread");
		if(c_freq_change_)
		{
			ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
			r = ros::Rate(controller_frequency_);
			c_freq_change_ = false;
		}
		publish = false;
		{
			boost::mutex::scoped_lock lock(mission_mtx_);
			if(has_new_mission_)
			{
				recevied_ms = recevied_ms_;
				has_new_mission_ = false;
			}
		}
		switch(recevied_ms)
		{
		case STOP:
		{

			{
				boost::mutex::scoped_lock lock(mission_mtx_);
				recevied_ms_ = STOP;
				publishZeroVelocity();
				std_msgs::String detector_control_msg;
				detector_control_msg.data = "00";
				lane_detector_pub_.publish(detector_control_msg);
				std_srvs::Empty::Request req;
				std_srvs::Empty::Response res;
				while(!stop_front_camera_client_.call(req,res))
				{
					ROS_ERROR("waiting for stop front camera!");
					ros::Rate(2).sleep();
				}
				while(!stop_rear_camera_client_.call(req,res))
				{
					ROS_ERROR("waiting for stop rear camera!");
					ros::Rate(2).sleep();
				}
				reset();
				mission_cond_.wait(mission_mtx_);
				recevied_ms = recevied_ms_;
				last_valid_control_ = ros::Time::now();
			}
			break;
		}
		case FORWARD:
		case BACKWARD:
		{
			// if we do not detector lane or we still not arrive at goal
			if(!getLinePoseFromCamera(line_pose) || sub_ms == REACH)
			{
				// if we never detect lane , we rotate robot to first left then right
				if(!init_line_pose_)
				{

					if(!findLane())
					{   // if we still not found the lane after we check left and right, we stop mission and report failed msg
						publish = true;
						motion_status_json[MSG_RESULT] = 7;
						publishZeroVelocity();
						//this->motion_status_pub_.publish(motion_status);
						recevied_ms = STOP;
					}
					else
					{
						// we simulate how far we moved without a real goal and calculate a goal from old info
						computeLinePose(line_pose);
					}
				}
				else //if at first we get a sub goal, then cant see lane, we simulate how far we moved without a real goal, and calculate a goal from old info
				{
					computeLinePose(line_pose);
				}
			}
			else // we found lane, we use the goal msg from camera
			{	
				if(sub_ms!=MOVE)
				{
					publishZeroVelocity();
					ros::Rate(1).sleep();	
					sub_ms = MOVE;
				}
				if(init_line_pose_!= true&&recevied_ms == FORWARD)
				{
					recordCurrentPose(out_start_pose_);
				}
				ROS_INFO("get goal_pose %f, %f, %f", line_pose.getOrigin().getX(), line_pose.getOrigin().getY(),  tf::getYaw(line_pose.getRotation()));
				init_line_pose_ = true;
			}
			// save last goal pose and time to simulate a new goal pose if after we can not see lane
			last_line_pose_ = line_pose;
			last_valid_plan_ = ros::Time::now();
			// send goal to planner
			geometry_msgs::PoseStamped pose_msg;
			tf::poseStampedTFToMsg(line_pose,pose_msg);
			global_plan_.clear();
			global_plan_.push_back(pose_msg);
			control_planner_->setPlan(global_plan_);

			boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

			if(sub_ms == ROTATION_LEFT) // if our mission is rotating left to find lane
			{
				geometry_msgs::Twist cmd_vel;
				//we find a speed
				if(control_planner_->computeVelocityCommands(cmd_vel))
				{
					if(control_planner_->isGoalReached())
					{
						publishZeroVelocity();
						has_check_left_ = true;
						ROS_INFO("check left finish");
					}else
					{
						check_vel_setup_sonar_mode(cmd_vel);
						vel_pub_.publish(cmd_vel);
					}
					last_valid_control_ = ros::Time::now();
				}
				else// we can not find a speed , thats means we find collision, we wait it disappear or time out
				{
					lock.unlock();
					publishZeroVelocity();
					ROS_ERROR("left waiting time %f",(ros::Time::now()-last_valid_control_).toSec());
					// time out, check left finish
					this->clearCostmap();
					if((ros::Time::now()-last_valid_control_).toSec()>controller_patience_/3.0 )
					{

						has_check_left_ = true;
						last_valid_control_ = ros::Time::now();
					}
					else
					{
						ros::Rate(2.0).sleep();
					}
				}


			}
			else if(sub_ms == ROTATION_RIGHT)// if our mission is rotating right to find lane
			{


				geometry_msgs::Twist cmd_vel;
				//we find a speed
				if(control_planner_->computeVelocityCommands(cmd_vel))
				{
					if(control_planner_->isGoalReached())
					{
						publishZeroVelocity();
						has_check_right_ = true;
						ROS_INFO("check right finish");
					}else
					{
						check_vel_setup_sonar_mode(cmd_vel);
						vel_pub_.publish(cmd_vel);
					}
					last_valid_control_ = ros::Time::now();
				}// we check collision
				else
				{
					lock.unlock();
					ROS_ERROR("right waiting time %f",(ros::Time::now()-last_valid_control_).toSec());
					publishZeroVelocity();

					// time out
					this->clearCostmap();
					if((ros::Time::now()-last_valid_control_).toSec()>controller_patience_/3.0 )
					{
						has_check_right_ = true;
						last_valid_control_ = ros::Time::now();
					}
					else
					{
						ros::Rate(2.0).sleep();
					}
				}

			}
			else if (sub_ms == REACH) // we should stop slowly
			{

				if(control_planner_->computeVelocityCommands(cmd_vel))
				{
					ROS_DEBUG("goal reached %f,%d",cmd_vel.linear.x,control_planner_->isGoalReached());
					if(control_planner_->isGoalReached())
					{
						publishZeroVelocity();
						// if we still can not find the new goal from camera when too close to the last goal, we stop this mission and go back to origin point
						// if we move to go, then we come back to origin
						if(execute_ms_ ==  BACKWARD&&recevied_ms== FORWARD)
						{
							motion_status_json[MSG_RESULT] = 8;
						}
						else if(execute_ms_ ==  FORWARD&&recevied_ms== FORWARD)
						{
							motion_status_json[MSG_RESULT] = 2;
						}else if(execute_ms_ ==  BACKWARD&&recevied_ms== BACKWARD)
						{
							motion_status_json[MSG_RESULT] = 4;
						}
						publish = true;
						recevied_ms= STOP;
						sub_ms = STOP;
						reset();
					}
					else
					{

						// if our mission is backward now, our velocity should be negativ
						if(execute_ms_ == BACKWARD)
						{
							cmd_vel.linear.x*=-1.0;
						}
						check_vel_setup_sonar_mode(cmd_vel);
						vel_pub_.publish(cmd_vel);
						last_valid_control_ = ros::Time::now();
					}// we are waiting for no collision
				}				
				else
				{

					publishZeroVelocity();
					// if we still can not find the new goal from camera when too close to the last goal, we stop this mission and go back to origin point
					// if we move to go, then we come back to origin
					if(execute_ms_ ==  BACKWARD&&recevied_ms== FORWARD)
					{
						motion_status_json[MSG_RESULT] = 8;
					}
					else if(execute_ms_ ==  FORWARD&&recevied_ms== FORWARD)
					{
						motion_status_json[MSG_RESULT] = 2;
					}else if(execute_ms_ ==  BACKWARD&&recevied_ms== BACKWARD)
					{
						motion_status_json[MSG_RESULT] = 4;
					}
					publish = true;
					recevied_ms= STOP;
					sub_ms = STOP;
					reset();
				}

			}
			else if(sub_ms == MOVE)// if our mission is following lane
			{
				if(!isTargetReached())// if we donot received final goal from camera,continue follow lane
				{
					geometry_msgs::Twist cmd_vel;
					//we find a speed
					if(control_planner_->computeVelocityCommands(cmd_vel))
					{
						/*if(getSubGoalDist()<min_sub_goal_dist_)
						{
							tf::Stamped<tf::Pose> local_pose;
						    try{
								tf_.transformPose("base_link",goal_pose_,local_pose);
								if(execute_ms_ ==  BACKWARD)
								{
									last_goal_pose_.getOrigin().setX(-1*local_pose.getOrigin().getX());
									last_goal_pose_.getOrigin().setY(-1*local_pose.getOrigin().getY());
									last_goal_pose_.setRotation(local_pose.getRotation());
								}
								else
								{
									last_goal_pose_.setOrigin(local_pose.getOrigin());
									last_goal_pose_.setRotation(local_pose.getRotation());
								}
						    }
						    catch(tf::TransformException& ex){
						      ROS_ERROR("Failed to transform the goal pose from %s into the base_link %s",
						    		  goal_pose_.frame_id_.c_str(), ex.what());
						    }
					    }
						 */

						if(control_planner_->isGoalReached())
						{

							publishZeroVelocity();
							// if we still can not find the new goal from camera when too close to the last goal, we stop this mission and go back to origin point
							// if we move to go, then we come back to origin
							if(execute_ms_ ==  BACKWARD)
							{
								publish = true;
								//this->motion_status_pub_.publish(motion_status);
								motion_status_json[MSG_RESULT] = 7;
								if((ros::Time::now()-last_valid_control_).toSec()>find_lane_waiting_time_ )
								{
									recevied_ms =  STOP;
									reset();
								}
							}
							else if(execute_ms_ == FORWARD)
							{
								std_msgs::String detector_control_msg;
								detector_control_msg.data = "01";
								lane_detector_pub_.publish(detector_control_msg);
								publish = true;
								motion_status_json[MSG_RESULT] = 3;
								execute_ms_ = BACKWARD;
								reset();

							}

						}
						else
						{
							// if our mission is backward now, our velocity should be negativ
							if(execute_ms_ == BACKWARD)
							{
								cmd_vel.linear.x*=-1.0;
							}
							check_vel_setup_sonar_mode(cmd_vel);
							vel_pub_.publish(cmd_vel);
							last_valid_control_ = ros::Time::now();
						}


					}// we are waiting for no collision
					else
					{
						lock.unlock();
						publishZeroVelocity();
						// time out
						ROS_ERROR("move waiting time %f",(ros::Time::now()-last_valid_control_).toSec());
						this->clearCostmap();
						if((ros::Time::now()-last_valid_control_).toSec()>controller_patience_ )
						{
							// if we are backward but time out, we should report the message
							if(execute_ms_ == BACKWARD)
							{
								publish = true;
								motion_status_json[MSG_RESULT] = 9;
							}
							else if(execute_ms_ == FORWARD)
							{
								std_msgs::String detector_control_msg;
								detector_control_msg.data = "01";
								lane_detector_pub_.publish(detector_control_msg);
								line_pose_= out_start_pose_;
								execute_ms_ = BACKWARD;

								publish = true;
								motion_status_json[MSG_RESULT] = 3;
								reset();
							}
							last_valid_control_ = ros::Time::now();

						}
						else
						{
							if(execute_ms_ == BACKWARD)
							{
								motion_status_json[MSG_RESULT] = 11;
							}
							else if(execute_ms_ == FORWARD)
							{
								motion_status_json[MSG_RESULT] = 10;
							}
							publish = true;
							ros::Rate(1.6).sleep();
						}
					}

				}
				else // we find the target
				{

					sub_ms = REACH;
					// if our mission is forward
					if(execute_ms_ ==  BACKWARD)
					{
						last_line_pose_.frame_id_ = "rear_frame";
						last_line_pose_.setIdentity();
						last_line_pose_.getOrigin().setX(max_stop_dist_);
					}
					else if(execute_ms_ ==  FORWARD)
					{
						last_line_pose_.frame_id_ = "front_frame";
						last_line_pose_.setIdentity();
						last_line_pose_.getOrigin().setX(max_stop_dist_);
					}
				}
			}

			break;
		}

		case PAUSE:
		{
			last_valid_control_ = ros::Time::now();
			publishZeroVelocity();
			break;
		}
		default:
		{
			recevied_ms = STOP;
		}
		}
		last_valid_plan_ = ros::Time::now();

		if(publish)
		{
			publish = false;
			Json::StyledWriter fast_motion;
			motion_status.data = fast_motion.write(motion_status_json);
			this->motion_status_pub_.publish(motion_status);
		}
		r.sleep();

		// info from camera
	}
}
bool LaneFollower::findLane()
{

	if(!has_check_left_)
	{

		if(!init_true_left_)
		{
			ROS_INFO("start check left");
			sub_ms = ROTATION_LEFT;
			last_line_pose_.frame_id_ = "front_frame";
			last_line_pose_.setIdentity();
			last_line_pose_.setRotation(tf::createQuaternionFromYaw(this->max_turn_left_angle_));
			init_true_left_ = true;

		}
	}
	else if(!has_check_right_)
	{
		if(!init_true_right_)
		{
			ROS_INFO("start check right");
			sub_ms = ROTATION_RIGHT;
			last_line_pose_.frame_id_ = "front_frame";
			last_line_pose_.setIdentity();
			last_line_pose_.setRotation(tf::createQuaternionFromYaw(-1*this->max_turn_right_angle_));
			init_true_right_ = true;
		}
	}
	else if(has_check_left_&&has_check_right_)
	{
		++current_rotation_repeat_;
		has_check_left_=false;
		has_check_right_=false;
		init_true_left_ = false;
		init_true_right_ = false;
		if(current_rotation_repeat_>max_rotation_repeat_)
		{
			current_rotation_repeat_ = 0;
			return false;
		}
	}
	return true;
}
double LaneFollower::angleClassfication(double angle)
{
	return (fabs(angle)<0.02?0.0:angle);
}
void LaneFollower::clearCostmap()
{
	std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = controller_costmap_ros_->getLayeredCostmap()->getPlugins();
	for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
		boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
		std::string name = plugin->getName();
		int slash = name.rfind('/');
		if( slash != std::string::npos ){
			name = name.substr(slash+1);
		}

		if(clearable_layers_.count(name)!=0){
			boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
			costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
			clearMap(costmap);
		}
	}
}
void LaneFollower::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap){
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

	unsigned char* grid = costmap->getCharMap();
	int x=(int)costmap->getSizeInCellsX();int y=(int)costmap->getSizeInCellsY();
	memset(grid,costmap_2d::FREE_SPACE,x*y*(sizeof(costmap_2d::FREE_SPACE)));
	double ox = costmap->getOriginX(), oy = costmap->getOriginY();
	double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
	costmap->addExtraBounds(ox, oy, ox + width, oy + height);
	return;
}
//get sub goal pose and orientation from lane detector node
bool LaneFollower::getLinePoseFromCamera(tf::Stamped<tf::Pose>& line_pose)
{
	boost::mutex::scoped_lock lock(goal_mtx);
	if(execute_ms_ == FORWARD)
	{
		if(this->received_front_line_)
		{
			//double last_angle = tf::getYaw(goal_pose.getRotation());
			//angle = std::fabs(last_angle - angle)>0.05?angle:last_angle;
			line_pose = new_front_pose_;

			this->received_front_line_ = false;
			return true;
		}
		else
		{
			return false;
		}

	}
	else
	{
		if(this->received_rear_line_)
		{
			line_pose = new_rear_pose_;
			this->received_rear_line_ = false;
			return true;
		}
		else
		{
			return false;
		}
	}
}
// if we didn't receive a new goal pose , we simulate a new goal from last goal and current velocity
void LaneFollower::computeLinePose(tf::Stamped<tf::Pose>& line_pose)
{

	geometry_msgs::Twist robot_vel;
	getRobotVel(robot_vel);
	double dt = (ros::Time::now() - last_valid_plan_).toSec();
	line_pose = last_line_pose_;
	if(execute_ms_ == BACKWARD)
	{
		robot_vel.linear.x*=-1;
	}
	double x=0.0, y=0.0, x_new=0.0, y_new=0.0, theta =0.0,th =0.0;
	x = line_pose.getOrigin().getX();
	y = line_pose.getOrigin().getY();

	//theta = tf::getYaw(last_goal_pose_.getRotation());
	//theta = std::acos(last_goal_pose_.getOrigin().getX()/hypot(last_goal_pose_.getOrigin().getX(),last_goal_pose_.getOrigin().getY()));
	//x = last_goal_pose_.getOrigin().getX();
	//y = last_goal_pose_.getOrigin().getY();
	//theta = tf::getYaw(last_goal_pose_.getRotation());
	//x = computeNewXPosition(x, robot_vel.linear.x, robot_vel.linear.y, th, dt);
	//y = computeNewYPosition(y, robot_vel.linear.x, robot_vel.linear.y, th, dt);
	//th += std::acos(goal_pose.getOrigin().getX()/hypot(goal_pose.getOrigin().getX(),goal_pose.getOrigin().getY())) ;
	//tf::Transform trans(tf::createQuaternionFromYaw(th),tf::Vector3(-x,-y,0));
	//goal_pose.operator *()  *= trans.getRotation() ;
	//goal_pose.getOrigin().rotate(tf::Vector3(0,0,1),th);
	//goal_pose.setOrigin(goal_pose.getOrigin()+trans.getOrigin());
	//theta = computeNewThetaPosition(theta,robot_vel.angular.z, dt);
	//x =(goal_pose.getOrigin().getX()-x)*std::cos(th);
	//y =(goal_pose.getOrigin().getY()-y)*std::sin(th);

	th = -robot_vel.angular.z * dt;
	double st = std::sin(th);
	double ct = std::cos(th);
	//
	x_new = x*ct-y*st;
	y_new = x*st+y*ct;
	x = x_new - robot_vel.linear.x*dt*ct;
	y = y_new - robot_vel.linear.y*dt*st;


	theta = tf::getYaw(line_pose.getRotation()) + th;


	line_pose.getOrigin().setX(x);
	line_pose.getOrigin().setY(y);
	line_pose.setRotation(tf::createQuaternionFromYaw(theta));
	ROS_DEBUG("compute goal_pose %f, %f, %f",line_pose.getOrigin().getX(), line_pose.getOrigin().getY(), tf::getYaw(line_pose.getRotation()));

}
// check if we arrive final goal from lane detector and amcl odom
bool LaneFollower::isTargetReached()
{
	bool reached = false;
	if(execute_ms_ == FORWARD)
	{
		reached =  this->find_front_target_;
	}
	else
	{
		reached =  this->find_rear_target_;
	}
	if(reached)
	{
		reached = isRecivedFinalGoalReached();
	}
	if(using_amcl_)
	{
		double x ,y;
		{
			boost::mutex::scoped_lock lock(odom_lock_);
			x = base_odom_.pose.pose.position.x;
			y = base_odom_.pose.pose.position.y;
		}
		if(hypot((line_pose_.getOrigin().x()-x),(line_pose_.getOrigin().y()-y))<min_global_goal_dist_)
		{
			ROS_INFO("amcl decided goal reached!");
			reached =  true;
		}
		else if(hypot((line_pose_.getOrigin().x()-x),(line_pose_.getOrigin().y()-y))>min_global_goal_dist_*2.0)
		{
			ROS_INFO("amcl decided goal not reached!");
			reached =  false;
		}
	}

	return reached;
}

bool LaneFollower::isRecivedFinalGoalReached()
{
	tf::Stamped<tf::Pose> current_pose;
	recordCurrentPose(current_pose);
	double moved_distance = (current_pose.getOrigin() - last_robot_pose_.getOrigin()).length();
	if(dist_to_final_goal_from_camera_ - moved_distance < camera_goal_toloranz_ )
	{
		return true;
	}
	return false;
}


void LaneFollower::publishZeroVelocity()
{
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.angular.z = 0.0;
	vel_pub_.publish(cmd_vel);
}
// reset all flag variables
void LaneFollower::reset()
{

	has_check_right_ = false;
	has_check_left_ = false;
	init_true_right_ = false;
	init_true_left_ = false;
	received_front_line_ = false;
	received_rear_line_ = false;
	init_line_pose_ = false;
	find_rear_target_ = false;
	find_front_target_ = false;
	current_rotation_repeat_ = 0; 
}

// if we move forward, we must record goal in case we cant move forward any more. we move backward
void LaneFollower::recordCurrentPose(tf::Stamped<tf::Pose>& current_pose )
{
	boost::mutex::scoped_lock lock(odom_lock_);
	current_pose.getIdentity();
	current_pose.frame_id_ = base_odom_.header.frame_id;
	current_pose.stamp_ = base_odom_.header.stamp;
	current_pose.getOrigin().setX(base_odom_.pose.pose.position.x);
	current_pose.getOrigin().setY(base_odom_.pose.pose.position.y);
	current_pose.getOrigin().setZ(base_odom_.pose.pose.position.z);
}
// get robot velocity from amcl
void LaneFollower::getRobotVel(geometry_msgs::Twist &global_vel) {
	// Set current velocities from odometry
	{
		boost::mutex::scoped_lock lock(odom_lock_);
		global_vel.linear.x = base_odom_.twist.twist.linear.x;
		global_vel.linear.y = base_odom_.twist.twist.linear.y;
		global_vel.angular.z = base_odom_.twist.twist.angular.z;
		//robot_vel.frame_id_ = base_odom_.child_frame_id;
	}
	//robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
	//robot_vel.stamp_ = ros::Time();
}

Json::Value LaneFollower::handleCmd(Json::Value cmd)
{
	Json::Value res;
	res[MSG_SUB_NAME] = cmd[MSG_PUB_NAME];

	if(CMD_LANE_FOLLOWING_PAUSE == cmd[MSG_PUB_NAME].asString())
	{
		//pause;
		if(FORWARD == recevied_ms_||BACKWARD == recevied_ms_)
		{
			last_ms_ = recevied_ms_;
			recevied_ms_ = PAUSE;
			res[MSG_ERROR_CODE] = 0;
			res[MSG_MSG] = "pause success";
		}
		else
		{
			res[MSG_ERROR_CODE] = 1;
			res[MSG_MSG] = "pause failed status is not right";
		}
		res[MSG_DATA] = "data";
	}
	else if(CMD_LANE_FOLLOWING_RESUME == cmd[MSG_PUB_NAME].asString())
	{
		//resume pause;
		if(recevied_ms_== PAUSE)
		{
			recevied_ms_ = last_ms_;
			res[MSG_ERROR_CODE] = 0;
			res[MSG_MSG] = "resume success";

		}
		else
		{
			res[MSG_ERROR_CODE] = 1;
			res[MSG_MSG] = "resume failed status is not right";
		}
		res[MSG_DATA] = "data";
	}
	else if(CMD_LANE_FOLLOWING_FORWARD == cmd[MSG_PUB_NAME].asString())
	{

		if(!cmd[MSG_DATA].isNull())
		{
			Json::Value data;
			data = cmd[MSG_DATA];
			ROS_INFO("forward");
			double yaw = 0.0;
			double x = 1.0;
			double y = 1.0;
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
			line_pose_.frame_id_ = "map";
			line_pose_.stamp_ = ros::Time::now();
			line_pose_.setRotation((tf::createQuaternionFromYaw(yaw)));
			line_pose_.getOrigin().setX(x);
			line_pose_.getOrigin().setY(y);

			/*		tf::Stamped<tf::Pose> local_pose;
		    try{
				tf_.transformPose("odom",goal_pose_,local_pose);
				goal_pose_ = local_pose;
		    }
		    catch(tf::TransformException& ex){
		      ROS_ERROR("Failed to transform the goal pose from %s into the base_link %s",
		    		  goal_pose_.frame_id_.c_str(), ex.what());
		    }*/
			std_msgs::String detector_control_msg;
			detector_control_msg.data = "10";
			lane_detector_pub_.publish(detector_control_msg);
			reset();
			execute_ms_ = FORWARD;
			recevied_ms_ = FORWARD;
			std_srvs::Empty::Request req;
			std_srvs::Empty::Response resp;
			while(!start_front_camera_client_.call(req,resp))
			{
				ROS_ERROR("waiting for start front camera!");
				ros::Rate(2).sleep();
			}
			while(!start_rear_camera_client_.call(req,resp))
			{
				ROS_ERROR("waiting for start rear camera!");
				ros::Rate(2).sleep();
			}
			mission_cond_.notify_one();
			std_msgs::String msg;
			res[MSG_DATA] = cmd[MSG_DATA];
			res[MSG_ERROR_CODE] = 0;
			res[MSG_MSG] = "start forward success";
			ROS_INFO("start forward success");
		}
		else
		{
			res[MSG_DATA] = cmd[MSG_DATA];
			res[MSG_ERROR_CODE] = 1;
			res[MSG_MSG] = "json invalid";
		}

	}
	else if(CMD_LANE_FOLLOWING_BACKWARD == cmd[MSG_PUB_NAME].asString())
	{
		if(!cmd[MSG_DATA].isNull())
		{
			Json::Value data;
			data = cmd[MSG_DATA];


			double yaw = 1.0;
			double x = 1.0;
			double y = 1.0;
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
			line_pose_.frame_id_ = "map";
			line_pose_.stamp_ = ros::Time::now();
			line_pose_.setRotation((tf::createQuaternionFromYaw(yaw)));
			line_pose_.getOrigin().setX(x);
			line_pose_.getOrigin().setY(y);
			/*		tf::Stamped<tf::Pose> local_pose;
	    try{
			tf_.transformPose("odom",goal_pose_,local_pose);
			goal_pose_ = local_pose;
	    }
	    catch(tf::TransformException& ex){
	      ROS_ERROR("Failed to transform the goal pose from %s into the base_link %s",
	    		  goal_pose_.frame_id_.c_str(), ex.what());
	    }*/
			std_msgs::String detector_control_msg;
			detector_control_msg.data = "01";
			lane_detector_pub_.publish(detector_control_msg);
			execute_ms_ = BACKWARD;
			recevied_ms_ = BACKWARD;
			std_srvs::Empty::Request req;
			std_srvs::Empty::Response resp;
			while(!start_front_camera_client_.call(req,resp))
			{
				ROS_ERROR("waiting for start front camera!");
				ros::Rate(2).sleep();
			}
			while(!start_rear_camera_client_.call(req,resp))
			{
				ROS_ERROR("waiting for start rear camera!");
				ros::Rate(2).sleep();
			}
			reset();
			mission_cond_.notify_one();
			res[MSG_DATA] = cmd[MSG_DATA];
			res[MSG_ERROR_CODE] = 0;
			res[MSG_MSG] = "start backward success";
		}
		else
		{
			res[MSG_DATA] = cmd[MSG_DATA];
			res[MSG_ERROR_CODE] = 1;
			res[MSG_MSG] = "json invalid";
		}
	}
	else if(CMD_LANE_FOLLOWING_CANCEL == cmd[MSG_PUB_NAME].asString())
	{

		//if((FORWARD == recevied_ms_) ||(BACKWARD == recevied_ms_)|| (PAUSE == recevied_ms_))
		//{
		recevied_ms_ = STOP;
		res[MSG_DATA] = "data";
		res[MSG_ERROR_CODE] = 0;
		res[MSG_MSG] = "cancel success";

		//}
		//else
		//{
		//   res[MSG_ERROR_CODE] = 1;
		//    res[MSG_MSG] = "cancel failed status is not right";
		//}
	}
	return res;
}
void LaneFollower::setup_sonar_mode(int mode)
{
    int actual_mode = 999;
    if(mode != actual_mode||!ros::param::get("/noah_sensors/ultrasonic_work_mode_ack", actual_mode))
    {
    	 ros::param::set("/noah_sensors/ultrasonic_work_mode", mode);
    }
    return;
}

int LaneFollower::get_sonar_mode()
{
    int actual_mode = 999;
    ros::param::get("/noah_sensors/ultrasonic_work_mode_ack", actual_mode);
    return actual_mode;
}

void LaneFollower::check_vel_setup_sonar_mode(const geometry_msgs::Twist &cmd_vel)
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

};


