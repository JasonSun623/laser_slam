#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <vector>
#include <string>
#include "laser_line_extraction/line_extraction.h"


namespace line_extraction
{

class LineExtractionROS
{

public:
  // Constructor / destructor
  LineExtractionROS();
  ~LineExtractionROS();

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::Subscriber scan_subscriber_;
  ros::Publisher scan_line_publisher_;
  // Parameters
  std::string frame_id_;
  std::string scan_topic_;
  bool pub_markers_;
  // Line extraction
  LineExtraction line_extraction_;

  // Members
  void loadParameters();
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&);
};

} // namespace line_extraction

#endif
