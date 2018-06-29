#include "laser_line_extraction/line_extraction_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_extraction_node");
  line_extraction::LineExtractionROS line_extractor;

  ros::spin();
  return 0;
}

