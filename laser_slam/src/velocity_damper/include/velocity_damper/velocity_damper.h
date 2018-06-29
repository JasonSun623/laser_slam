/*
 * velocity_damper.h
 *
 *  Created on: Mar 24, 2017
 *      Author: tomzhang
 */

#ifndef INCLUDE_VELOCITY_DAMPER_VELOCITY_DAMPER_H_
#define INCLUDE_VELOCITY_DAMPER_VELOCITY_DAMPER_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
namespace velocity_damper
{
class
VelocityDamper{
public :
	VelocityDamper(std::string name, tf::TransformListener *tf);
	~VelocityDamper();
	void onInitialize();
	void setupDynamicReconfigure(ros::NodeHandle& nh);
	void reconfigureCB();
	void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message);
	void velocityCallback(const geometry_msgs::TwistConstPtr & message);
protected:
	  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_; ///< @brief Used for the observation message filters
	  //std::vector<boost::shared_ptr<velocity_damper::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors
	  std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor

private:
	std::string name_;
    double factor_;
	tf::TransformListener *tf_;
	boost::mutex mtx_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	std::vector<double> avg_mean_filter_;
	bool filter_init_;
};

}//namespace

#endif /* INCLUDE_VELOCITY_DAMPER_VELOCITY_DAMPER_H_ */
