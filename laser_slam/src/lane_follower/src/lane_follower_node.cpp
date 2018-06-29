/*
 * lane_follower_node.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: tomzhang
 */



#include "lane_follower/lane_follower.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "lane_follower_node");
  tf::TransformListener tf(ros::Duration(10));

  lane_follower::LaneFollower lane_follower( tf );

  ros::spin();

  return(0);
}

