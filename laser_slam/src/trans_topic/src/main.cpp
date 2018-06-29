#include "ros/ros.h"
#include "ros/master.h"
#include "ros/network.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Joy.h"

#include <sstream>
#include <math.h>
#include <stdio.h>
#include <vector>

ros::Publisher app_pub_pub;
ros::Publisher app_sub_pub;

void app_pub_callback(const std_msgs::String::ConstPtr& msg)
{
    app_pub_pub.publish(*msg);
}

void app_sub_callback(const std_msgs::String::ConstPtr& msg)
{
    app_sub_pub.publish(*msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "trans_topic");
    ros::NodeHandle n;

    ros::Subscriber app_pub_sub = n.subscribe("app_pub",1000,app_pub_callback);
    app_pub_pub = n.advertise<std_msgs::String>("app_nav_pub",1000);

    ros::Subscriber app_sub_sub = n.subscribe("app_nav_sub",1000,app_sub_callback);
    app_sub_pub = n.advertise<std_msgs::String>("app_sub",1000);

    ros::Rate r(50);
    int count = 0;
    while(ros::ok())
    {
        if(0 == (count%100))
        {
            ROS_DEBUG("trans_topic is running");
        }
        count++;

        if(ros::master::check())
        {
            ROS_DEBUG("check true");
        }
        else
        {
            ROS_ERROR("check false");
        }
        r.sleep();
        ros::spinOnce();
    }

}
