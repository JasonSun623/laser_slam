#ifndef UWB_LOCATION_NODE_H_
#define UWB_LOCATION_NODE_H_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <vector>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "map_server/query_uwb.h"
#include "map_server/point.h"
#include "uwb_location/uart.h"
#include "uwb_location/Anchor.h"
#include "uwb_location/UwbAnchors.h"

#define DEVICE_NAME_LEN (50)
#define BUF_LEN (256)
#define DATA_LEN (10)
#define ANCHOR_NUM (4)

typedef enum{
    COM_OPENING = 1,
    COM_CHECK_VERSION,
    COM_RUN_OK,
    COM_CLOSING,
}com_state_e;

typedef struct{
    unsigned char uwb_status;   
    unsigned char uwb_rssi;      
    unsigned char cmd;
    int uwb_open_station;
    int handle_data_flag;
    int work_normal;
    int com_rssi;       //the interface judge com communication
    int rec_num;
    com_state_e com_state;
    int com_device;
    char dev[DEVICE_NAME_LEN];
}uwb_sys_t;

typedef struct{
    int id;   
    double distance;
}anchor_t;


class UwbLocationNode
{
	public:
		UwbLocationNode();
		~UwbLocationNode();
        void odom_received(const nav_msgs::Odometry& msg);
        void current_pose_received(const nav_msgs::Odometry& msg);
        
	private:
        void update_system_state();
        int handle_receive_data();
        void uwb_driver_thread();
        void uwb_compute_thread();
		void pub_uwb_info();
        void set_uwb_pose(double x, double y, double z);
        void timer_callback(const ros::TimerEvent &e);
        bool is_current_pose_in_uwb(geometry_msgs::Pose pose);

		ros::NodeHandle nh_;
		ros::Publisher uwb_pose_pub;
		ros::Publisher uwb_odom_pub;
		ros::Publisher uwb_odom_origin_pub;
        ros::Publisher initial_pose_pub;
        ros::Publisher uwb_status_pub;
        ros::Publisher uwb_anchors_pub;
		ros::Subscriber current_pose_sub;
        ros::Subscriber odom_sub;
        ros::ServiceClient nomotion_client;
        ros::ServiceClient query_uwb_client;
		nav_msgs::Odometry uwb_odom;
        geometry_msgs::Pose robot_current_pose;
        geometry_msgs::Pose odom_pose;
        geometry_msgs::Pose uwb_pose;
        geometry_msgs::Pose latest_odom_pose;
        geometry_msgs::Pose latest_current_pose;
        geometry_msgs::Pose latest_uwb_pose;
        geometry_msgs::Pose first_uwb_pose;
        geometry_msgs::Pose pre_uwb_pose;
        geometry_msgs::Pose timer_odom_pose;
        geometry_msgs::Pose timer_uwb_pose;
        geometry_msgs::Pose timer_latest_odom_pose;
        geometry_msgs::Pose timer_latest_uwb_pose;
        bool timer_first;
        double timer_offset;
        geometry_msgs::PoseWithCovarianceStamped uwb_pose_with_cov;
        boost::mutex mutex_;
        ros::Timer uwb_timer;
        double update_rate;
        double update_min_distance;
        double cov_x;
        double cov_y;
        double cov_th;
        double max_offset_with_currentpose;
        double timer_max_offset;
        double timer_duration;
        bool is_use_map;
        bool is_compute_th;
        bool compute_th_ok;
        bool first_uwb_flag;
        bool is_use_uwb;
        bool is_uwb_good_self;
        bool is_uwb_good_compare_scan;
        double diff_range;
        double max_diff_range_self;
        tf::Transform transform_uwb;
        double uwb_map_th;
        double uwb_map_ths[2];
        int count_compute;
        std::vector<map_server::point> vec;

		uwb_sys_t uwb_sys;
        UwbSerial uwbSerial;
        int last_unread_bytes;
        unsigned char recv_buf_last[BUF_LEN];
        std::ofstream uwb_file;

        anchor_t anchors[ANCHOR_NUM];
        uwb_location::UwbAnchors uwb_anchors;
};

#endif

