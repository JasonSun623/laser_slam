#ifndef LINE_EXTRACTION_H
#define LINE_EXTRACTION_H

#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include "laser_line_extraction/utilities.h"
#include "laser_line_extraction/line.h"
#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"

namespace line_extraction
{

class LineExtraction
{

public:
  // Constructor / destructor
  LineExtraction();
  ~LineExtraction();
  // get scan line
  sensor_msgs::LaserScan getScanLine(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  // Parameter setting
  void setBearingVariance(double);
  void setRangeVariance(double);
  void setLeastSqAngleThresh(double);
  void setLeastSqRadiusThresh(double);
  void setMaxLineGap(double);
  void setMinLineLength(double);
  void setMinLinePoints(unsigned int);
  void setMinRange(double);
  void setMinSplitDist(double);
  void setOutlierDist(double);
  void setMaxDistanceInLine(double);

private:
  // Data structures
  CachedData c_data_;
  RangeData r_data_;
  Params params_;
  // Indices after filtering
  std::vector<unsigned int> filtered_indices_;
  // Line data
  std::vector<Line> lines_;
  boost::mutex scan_mutex_;
  sensor_msgs::LaserScan scan_rev;
  // Methods
  // Run
  void extractLines(std::vector<Line>&);
  // Data setting
  void   setCachedData(const std::vector<double>&, const std::vector<double>&,
                     const std::vector<double>&, const std::vector<unsigned int>&);
  void   setRangeData(const std::vector<double>&);
  double chiSquared(const Eigen::Vector2d&, const Eigen::Matrix2d&,
                    const Eigen::Matrix2d&);
  double distBetweenPoints(unsigned int index_1, unsigned int index_2);
  void   filterClosePoints();
  void   filterOutlierPoints();
  void   filterLines();
  void   mergeLines();
  void   split(const std::vector<unsigned int>&);
  bool   isPointInLine(double th, double range, const std::vector<Line>&);
  void   cacheData(const sensor_msgs::LaserScan::ConstPtr&);
};

} // namespace line_extraction

#endif
