#include "laser_line_extraction/line_extraction_ros.h"
#include <cmath>
#include <ros/console.h>


namespace line_extraction
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionROS::LineExtractionROS():nh_local_("~")
{
    loadParameters();
    scan_line_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan_lines", 1);
    scan_subscriber_ = nh_.subscribe(scan_topic_, 1, &LineExtractionROS::laserScanCallback, this);
}

LineExtractionROS::~LineExtractionROS()
{
}

///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::loadParameters()
{
  
  ROS_DEBUG("*************************************");
  ROS_DEBUG("PARAMETERS:");

  // Parameters used by this node
  
  std::string frame_id, scan_topic;
  bool pub_markers;

  nh_local_.param<std::string>("frame_id", frame_id, "laser");
  frame_id_ = frame_id;
  ROS_DEBUG("frame_id: %s", frame_id_.c_str());

  nh_local_.param<std::string>("scan_topic", scan_topic, "scan");
  scan_topic_ = scan_topic;
  ROS_DEBUG("scan_topic: %s", scan_topic_.c_str());

  nh_local_.param<bool>("publish_markers", pub_markers, false);
  pub_markers_ = pub_markers;
  ROS_DEBUG("publish_markers: %s", pub_markers ? "true" : "false");

  // Parameters used by the line extraction algorithm

  double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
          max_line_gap, min_line_length, min_range, min_split_dist, outlier_dist;
  double max_distance_in_line;
  int min_line_points;

  nh_local_.param<double>("bearing_std_dev", bearing_std_dev, 1e-3);
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
  ROS_DEBUG("bearing_std_dev: %f", bearing_std_dev);

  nh_local_.param<double>("range_std_dev", range_std_dev, 0.02);
  line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
  ROS_DEBUG("range_std_dev: %f", range_std_dev);

  nh_local_.param<double>("least_sq_angle_thresh", least_sq_angle_thresh, 1e-4);
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
  ROS_DEBUG("least_sq_angle_thresh: %f", least_sq_angle_thresh);
  
  nh_local_.param<double>("least_sq_radius_thresh", least_sq_radius_thresh, 1e-4);
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
  ROS_DEBUG("least_sq_radius_thresh: %f", least_sq_radius_thresh);

  nh_local_.param<double>("max_line_gap", max_line_gap, 0.4);
  line_extraction_.setMaxLineGap(max_line_gap);
  ROS_DEBUG("max_line_gap: %f", max_line_gap);

  nh_local_.param<double>("min_line_length", min_line_length, 0.5);
  line_extraction_.setMinLineLength(min_line_length);
  ROS_DEBUG("min_line_length: %f", min_line_length);

  nh_local_.param<double>("min_range", min_range, 0.4);
  line_extraction_.setMinRange(min_range);
  ROS_DEBUG("min_range: %f", min_range);

  nh_local_.param<double>("min_split_dist", min_split_dist, 0.05);
  line_extraction_.setMinSplitDist(min_split_dist);
  ROS_DEBUG("min_split_dist: %f", min_split_dist);

  nh_local_.param<double>("outlier_dist", outlier_dist, 0.05);
  line_extraction_.setOutlierDist(outlier_dist);
  ROS_DEBUG("outlier_dist: %f", outlier_dist);

  nh_local_.param<int>("min_line_points", min_line_points, 9);
  line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
  ROS_DEBUG("min_line_points: %d", min_line_points);

  nh_local_.param<double>("max_distance_in_line", max_distance_in_line, 0.005);
  line_extraction_.setMaxDistanceInLine(max_distance_in_line);
  ROS_DEBUG("max_distance_in_line: %f", max_distance_in_line);

  ROS_DEBUG("*************************************");
}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  sensor_msgs::LaserScan scan_line = line_extraction_.getScanLine(scan_msg);
  scan_line_publisher_.publish(scan_line);
}

} // namespace line_extraction

