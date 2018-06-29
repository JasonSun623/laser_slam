/*
 * lane_follower.h
 *
 *  Created on: Jul 12, 2017
 *      Author: tomzhang
 */

#ifndef INCLUDE_LANE_FOLLOWER_LANE_FOLLOWER_H_
#define INCLUDE_LANE_FOLLOWER_LANE_FOLLOWER_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <pluginlib/class_loader.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_layer.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <nav_msgs/Odometry.h>
#include "json/json.h"
#include "motion_planner/MotionOp.h"
namespace lane_follower{

#define MSG_PUB_NAME     ("pub_name")
#define MSG_SUB_NAME     ("sub_name")
#define MSG_ERROR_CODE   ("error_code")
#define MSG_MSG          ("msg")
#define MSG_DATA         ("data")
#define MSG_RESULT         ("result")
#define MOVE_LANE_FOLLOWING_MOTION   ("lane_following_motion")
#define CMD_LANE_FOLLOWING_FORWARD   ("lane_following_forward")
#define CMD_LANE_FOLLOWING_BACKWARD  ("lane_following_backward")
#define CMD_LANE_FOLLOWING_CANCEL    ("lane_following_cancel")
#define CMD_LANE_FOLLOWING_PAUSE     ("lane_following_pause")
#define CMD_LANE_FOLLOWING_RESUME    ("lane_following_resume")


enum EMission {STOP = 0,BACKWARD,FORWARD,MOVE,PAUSE,RESUME,REACH,ROTATION_LEFT,ROTATION_RIGHT};



class LaneFollower
{
public:
	LaneFollower(tf::TransformListener& tf);
	~LaneFollower();
	void executeMS();
	void cmdService();
	bool missionService(motion_planner::MotionOp::Request  &req,
			motion_planner::MotionOp::Response &res);
private:


	//void reconfigureCB(LaneFollower::MoveBaseConfig &config, uint32_t level);
	bool isTargetReached();
	void executeCycle();
	bool findLane();
	double angleClassfication(double angle);
	Json::Value handleCmd(Json::Value cmd);
	void reset();
	bool getLinePoseFromCamera(tf::Stamped<tf::Pose>& line_pose);
	void publishZeroVelocity();
	bool isRecivedFinalGoalReached();
	void computeLinePose(tf::Stamped<tf::Pose>& goal_pose);
	void receiveLineCallback(const std_msgs::String::ConstPtr &msg);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void getRobotVel(geometry_msgs::Twist &global_vel);
	void recordCurrentPose(tf::Stamped<tf::Pose>& current_pose );
	void clearCostmap();

	void setup_sonar_mode(int mode);


	int get_sonar_mode();

	void check_vel_setup_sonar_mode(const geometry_msgs::Twist &cmd_vel);

    void clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap);
	tf::TransformListener& tf_;
	pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
	boost::shared_ptr<nav_core::BaseLocalPlanner> control_planner_;
	costmap_2d::Costmap2DROS *controller_costmap_ros_;
    std::set<std::string> clearable_layers_;
	ros::Publisher current_goal_pub_, vel_pub_, lane_detector_pub_,motion_status_pub_;
	ros::Subscriber lane_detector_sub_;
	ros::ServiceServer service_;
	ros::ServiceClient start_front_camera_client_,stop_front_camera_client_,start_rear_camera_client_,stop_rear_camera_client_;
	std::vector<geometry_msgs::PoseStamped> global_plan_;
	tf::Stamped<tf::Pose> last_line_pose_, new_front_pose_,new_rear_pose_,out_start_pose_,line_pose_, last_robot_pose_;
	EMission recevied_ms_, last_ms_, sub_ms, execute_ms_;
	ros::Time last_valid_plan_, last_valid_control_;
	double controller_frequency_, inscribed_radius_, circumscribed_radius_;
	// max waiting time for collision
	double controller_patience_,max_stop_dist_;
	double rotation_angle_,max_turn_left_angle_,max_turn_right_angle_,gradient_;
	bool has_check_right_, has_check_left_,init_true_left_,init_true_right_, received_front_line_,received_rear_line_;
	bool setup_, p_freq_change_, c_freq_change_, stop_thread_, has_new_mission_;
	bool init_line_pose_, find_front_target_,find_rear_target_,using_amcl_;
    double pre_camera_pose_x_,pre_camera_pose_y_,pre_camera_pose_z_,pre_camera_pose_pitch_;
    double back_camera_pose_x_,back_camera_pose_y_,back_camera_pose_z_,back_camera_pose_pitch_;
    double min_global_goal_dist_,camera_goal_toloranz_,find_lane_waiting_time_,dist_to_final_goal_from_camera_;
	double front_pixel_heigth_,front_pixel_width_,back_pixel_heigth_,back_pixel_width_;
	int image_width_,image_height_,max_rotation_repeat_,current_rotation_repeat_;
	std::string front_frame,rear_frame;
	std::string robot_base_frame_;
	boost::recursive_mutex configuration_mutex_;
	boost::condition mission_cond_;
	boost::thread mission_thread_;
    boost::mutex odom_lock_,mission_mtx_,goal_mtx;
    ros::Subscriber odom_sub_;
    nav_msgs::Odometry base_odom_;
    bool initialized_;

    inline double computeNewXPosition(double x, double vx, double vy, double theta, double dt){
      return x + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
    }

    /**
     * @brief  Compute y position based on velocity
     * @param  yi The current y position
     * @param  vx The current x velocity
     * @param  vy The current y velocity
     * @param  theta The current orientation
     * @param  dt The timestep to take
     * @return The new y position
     */
    inline double computeNewYPosition(double y, double vx, double vy, double theta, double dt){
      return y + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
    }
    inline double computeNewThetaPosition(double theta, double vth, double dt){
      return theta + vth * dt;
    }
};

};


#endif /* INCLUDE_LANE_FOLLOWER_LANE_FOLLOWER_H_ */
