/*
 * velocity_damper_node.cpp
 *
 *  Created on: Mar 29, 2017
 *      Author: tomzhang
 */

#include <ros/ros.h>
#include "velocity_damper/velocity_damper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_damper");
  tf::TransformListener tf(ros::Duration(10));
  velocity_damper::VelocityDamper lcr("velocity_damper", &tf);

  ros::spin();

  return (0);
}
