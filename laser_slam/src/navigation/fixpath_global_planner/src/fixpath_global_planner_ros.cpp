#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include "fixpath_global_planner/fixpath_global_planner.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "fixpath_planner");
	ROS_ERROR("fixpath_planner ROS init");
	fixpath_global_planner::FixpathGlobalPlanner fixpath_planner();

	ros::spin();

	return(0);
}
