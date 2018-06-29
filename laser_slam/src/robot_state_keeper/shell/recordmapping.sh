#!/bin/bash

source /opt/ros/indigo/setup.bash
source /home/robot/catkin_ws/install/setup.bash

export ROS_PACKAGE_PATH=/home/robot/ros_ws:/home/robot/catkin_ws:$ROS_PACKAGE_PATH
export ROS_IP=192.168.3.51
export ROS_HOSTNAME=$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:11311

rosnode kill /amcl
rosnode kill /slam_gmapping

rosbag record /tf /scan /odom /scan_odom 
