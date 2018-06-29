#!/bin/bash

source /opt/ros/indigo/setup.bash
source /home/robot/catkin_ws/install/setup.bash

export ROS_PACKAGE_PATH=/home/robot/ros_ws:/home/robot/catkin_ws:$ROS_PACKAGE_PATH
export ROS_IP=192.168.3.51
export ROS_HOSTNAME=$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:11311

sudo rm -rf /home/robot/.ros/log

roscore &
sleep 3
roslaunch robot_state_keeper robotstate.launch &
sleep 3
roslaunch robot_state_keeper devices.launch &
sleep 5
roslaunch robot_state_keeper localization.launch &
sleep 5
roslaunch robot_state_keeper motion.launch &
sleep 5
roslaunch robot_state_keeper mapcenter.launch &
sleep 3
roslaunch robot_state_keeper charging.launch &


#roslaunch usb_cam usb_cam-test.launch &
#sleep 3
#roslaunch ar_track_alvar pr2_indiv_no_kinect.launch &
#sleep 2
#roslaunch starline mapping_sick.launch

#roslaunch starline combine_sick.launch &
#roslaunch robot_state_keeper combine_sick.launch &
