#include "yaml-cpp/yaml.h"
#include "motion_planner/motion_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");

    MotionPlanner planner;
    ros::spin();
    return 0;
}
