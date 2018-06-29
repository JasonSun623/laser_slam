#include "yaml-cpp/yaml.h"
#include "robot_state_keeper/robot_state_keeper.h"



int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_state_keeper");
    RobotStateKeeper keeper;
    
	ROS_INFO("robot state keeper");
	ros::spin();

	return 0;
}

