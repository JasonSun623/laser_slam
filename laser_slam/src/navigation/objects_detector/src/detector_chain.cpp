/*
 * detector_chain.cpp
 *
 *  Created on: Feb 7, 2018
 *      Author: tomzhang
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "message_filters/subscriber.h"
#include <boost/thread/mutex.hpp>
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"

class DetectorChain
{
protected:
	// Our NodeHandle
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// Components for tf::MessageFilter
	tf::TransformListener *tf_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
	tf::MessageFilter<sensor_msgs::LaserScan> *tf_filter_;
	double tf_filter_tolerance_;

	// Filter Chain
	filters::FilterChain<sensor_msgs::LaserScan> detector_chain_;

	// Components for publishing
	sensor_msgs::LaserScan msg_;
	std::vector<sensor_msgs::LaserScan> msgs_;
	ros::Publisher output_pub_;
	boost::mutex mtx;
	//boost::mutex::scoped_lock lock;
	// Deprecation helpers
	//ros::Timer deprecation_timer_;
	//bool  using_detector_chain_deprecated_;

public:
	// Constructor
	DetectorChain() :
		private_nh_("~"),
		scan_sub_(nh_, "/scan_filtered", 1),
		tf_(NULL),
		tf_filter_(NULL),
		//lock(mtx),
		detector_chain_("sensor_msgs::LaserScan")
{

		// Configure filter chain

		//using_detector_chain_deprecated_ = private_nh_.hasParam("detector_chain");

		//if (using_detector_chain_deprecated_)
			detector_chain_.configure("objects_detector_chain", private_nh_);
		//else
		//	detector_chain_.configure("scan_filter_chain", private_nh_);

		std::string tf_message_filter_target_frame;

		if (private_nh_.hasParam("tf_message_filter_target_frame"))
		{
			private_nh_.getParam("tf_message_filter_target_frame", tf_message_filter_target_frame);

			private_nh_.param("tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

			tf_ = new tf::TransformListener();
			tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_sub_, *tf_, "", 1);
			tf_filter_->setTargetFrame(tf_message_filter_target_frame);
			tf_filter_->setTolerance(ros::Duration(tf_filter_tolerance_));

			// Setup tf::MessageFilter generates callback
			tf_filter_->registerCallback(boost::bind(&DetectorChain::callback, this, _1));
		}
		else
		{
			// Pass through if no tf_message_filter_target_frame
			scan_sub_.registerCallback(boost::bind(&DetectorChain::callback, this, _1));
		}

		// Advertise output
		output_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_objects", 1000);

		// Set up deprecation printout
		//deprecation_timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(&DetectorChain::deprecation_warn, this, _1));
		ROS_DEBUG("detector chain init ok");
}

	// Destructor
	~DetectorChain()
	{
		if (tf_filter_)
			delete tf_filter_;
		if (tf_)
			delete tf_;
	}

	// Deprecation warning callback
	void deprecation_warn(const ros::TimerEvent& e)
	{
		//if (using_detector_chain_deprecated_)
			ROS_WARN("Use of '~detector chain' parameter in scan_to_scan_filter_chain has been deprecated. Please replace with '~detector chain'.");
	}

	// Callback
	void callback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
	{
		// Run the filter chain
		boost::mutex::scoped_lock lock(mtx);

		ROS_DEBUG("detector chain have scan msg");
		if(!msgs_.empty())
		{
			msgs_.clear();
		}
		msgs_.push_back(*msg_in);
		lock.unlock();
	}
	void update()
	{
		boost::mutex::scoped_lock lock(mtx);
		if(!msgs_.empty())
		{
			sensor_msgs::LaserScan msg_in,msg_out;
			msg_in = msgs_.back();
			lock.unlock();
			if (detector_chain_.update(msg_in, msg_out))
			{
				ROS_DEBUG("detector chain update scan OK");
				//only publish result if filter succeeded
				output_pub_.publish(msg_out);

			}
		}
		else
		{
			return;
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detector_chain");

	ROS_DEBUG("detector chain init start");
	DetectorChain t;
	ros::Rate r(20);
	while(ros::ok())
	{
		t.update();
		r.sleep();
		ros::spinOnce();
	}
	return 0;
}


