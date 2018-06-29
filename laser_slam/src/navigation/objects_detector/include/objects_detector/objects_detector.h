/*
 * objects_detector.h
 *
 *  Created on: Jan 15, 2018
 *      Author: tomzhang
 */

#ifndef NAVIGATION_LAYERS_OBJECTS_DETECTOR_INCLUDE_OBJECTS_DETECTOR_OBJECTS_DETECTOR_H_
#define NAVIGATION_LAYERS_OBJECTS_DETECTOR_INCLUDE_OBJECTS_DETECTOR_OBJECTS_DETECTOR_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <angles/angles.h>
#include <filters/filter_base.h>
#include "json/json.h"
static const std::string OBJECTS = "objects";
static const std::string FRAME_ID = "frame_id";
static const std::string STAMP = "stamp";
static const std::string TYPE = "type";
static const std::string FOOTPRINT = "footprint";
static const std::string DURATION = "duration";
static const std::string RADIUS = "radius";
static const std::string CENTER = "center";
namespace objects_detector
{

bool interval( double number, double lower, double upper );
double getEulerDistance(tf::Stamped<tf::Pose> &point1, tf::Stamped<tf::Pose> &point2);


struct SickbedLimits
{
	double leg_width_upper,leg_width_lower;
	double leggap_width_upper,leggap_width_lower;
	double leggap_length_upper,leggap_length_lower;
	double dot_limit;
	bool check_parallel;
};
struct Leg
{
	Leg(tf::Stamped<tf::Pose> &left, tf::Stamped<tf::Pose> &right,unsigned int left_idx, unsigned int right_id, bool is_virtual)
	{
		this->left = left;
		this->right = right;
		center.frame_id_ = left.frame_id_;
		center.stamp_ = left.stamp_;
		center.setIdentity();
		center.setOrigin((left.getOrigin()+right.getOrigin())*0.5);
		double th = std::atan2((left.getOrigin().y()-right.getOrigin().y()),(left.getOrigin().x()-right.getOrigin().x()));
		center.setRotation(tf::createQuaternionFromYaw(angles::normalize_angle(th)));
		this->left_idx = left_idx;
		this->right_idx = right_idx;
		this->is_virtual = is_virtual;
	};
	Leg(const Leg &leg)
	{
		this->left = leg.left;
		this->center = leg.center;
		this->right = leg.right;
		this->left_idx = leg.left_idx;
		this->right_idx = leg.right_idx;
		this->is_virtual = leg.is_virtual;
	};
	bool operator==(const Leg &other) const
	{
		return (((left.getOrigin() == other.left.getOrigin())&&(right.getOrigin() == other.right.getOrigin())));
	};
	tf::Stamped<tf::Pose> center, left, right;
	unsigned int left_idx,right_idx;
	bool is_virtual;
};
struct LegPaar
{
	LegPaar()
	{
		leg1 = NULL;
		leg2 = NULL;
	}
	LegPaar(const LegPaar &legpaar)
	{
		this->leg1 = legpaar.leg1;
		this->center = legpaar.center;
		this->leg2 = legpaar.leg2;
	};
	LegPaar(Leg *leg1, Leg *leg2)
	{
		this->leg1 = leg1;
		this->leg2 = leg2;
		//ROS_ERROR("1");
		//ROS_ERROR("%s",this->leg1->center.frame_id_.c_str());
		//ROS_ERROR("%f",this->leg1->center.stamp_.toSec());
		center.frame_id_ = this->leg1->center.frame_id_;
		center.stamp_ = this->leg1->center.stamp_;

		//ROS_ERROR("%s",center.frame_id_.c_str());
		//ROS_ERROR("%f",center.stamp_.toSec());
		//ROS_ERROR("2");
		center.setIdentity();
		center.setOrigin((this->leg1->center.getOrigin()+this->leg2->center.getOrigin())*0.5);
		//ROS_ERROR("3");
		double th = std::atan2((this->leg1->center.getOrigin().y()-this->leg2->center.getOrigin().y()),(this->leg1->center.getOrigin().x()-this->leg2->center.getOrigin().x()));
		//ROS_ERROR("4");
		center.setRotation(tf::createQuaternionFromYaw(angles::normalize_angle(th)));
		//ROS_ERROR("5");
	}
	LegPaar& operator=(const LegPaar &paar)
	{
		this->leg1 = paar.leg1;
		this->leg2 = paar.leg2;
		this->center = paar.center;
		return *this;
	}
	tf::Stamped<tf::Pose> center;
	Leg* leg1;
	Leg* leg2;
};


struct Sickbed
{
	Sickbed(LegPaar &paar1,LegPaar &paar2, int type)
	{
		if(paar1.leg1->center.getOrigin().distance(paar2.leg1->center.getOrigin())>paar1.leg1->center.getOrigin().distance(paar2.leg2->center.getOrigin()))
		{
			std::swap(paar1.leg1,paar1.leg2);
		}

		this->paar1 = paar1;
		this->paar2 = paar2;


		center.frame_id_ = this->paar1.center.frame_id_;
		center.stamp_ = this->paar1.center.stamp_;
		center.setIdentity();
		center.setOrigin((this->paar1.center.getOrigin()+this->paar2.center.getOrigin())*0.5);
		//((this->paar1.center.getOrigin().y()-this->paar1.center.getOrigin().y())/(this->paar1.center.getOrigin().x()-this->paar1.center.getOrigin().x()));
		//center.setRotation(tf::createQuaternionFromYaw(std::atan2(center.getOrigin().getX(),center.getOrigin().getY())));

		double th = std::atan2((this->paar1.center.getOrigin().y()-this->paar2.center.getOrigin().y()),(this->paar1.center.getOrigin().x()-this->paar2.center.getOrigin().x()));
		center.setRotation(tf::createQuaternionFromYaw(angles::normalize_angle(th+M_PI*0.5)));
		this->type = type;
		legs.push_back(this->paar1.leg1);
		legs.push_back(this->paar1.leg2);
		legs.push_back(this->paar2.leg1);
		legs.push_back(this->paar2.leg2);
	}
	Sickbed(const Sickbed &sickbed)
	{
		this->paar1 = sickbed.paar1;
		this->center = sickbed.center;
		this->paar2 = sickbed.paar2;
		this->type = type;
		legs.push_back(this->paar1.leg1);
		legs.push_back(this->paar1.leg2);
		legs.push_back(this->paar2.leg1);
		legs.push_back(this->paar2.leg2);

	};
	tf::Stamped<tf::Pose> center;
	LegPaar paar1;
	LegPaar paar2;
	std::vector<Leg*> legs;
	int type;
};





class ObjectsDetector:public filters::FilterBase<sensor_msgs::LaserScan>
{

public:

	ObjectsDetector();

	~ObjectsDetector();

	void detect();

	void laserCallback (const sensor_msgs::LaserScan::ConstPtr& msg);

	bool configure();

    bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);


private:

	void meanFilter( sensor_msgs::LaserScan &laser_msg, unsigned size );

	void clustering(std::vector<int> &clusters,unsigned int &num_of_clusters,sensor_msgs::LaserScan &laser_msg);

	void checkCluster();

	void detectSickBed(std::vector<int> &clusters, sensor_msgs::LaserScan &laser_msg,SickbedLimits &limits);

	void findSickbedFeet(std::vector<int> &clusters, sensor_msgs::LaserScan &laser_msg, std::vector<Leg> &leg_candidates,SickbedLimits &limits);

	void matchingBedlegs(std::vector<Leg> &leg_candidates,SickbedLimits &limits);

	void machingRestPaar(std::vector<Leg> &leg_candidates,SickbedLimits &limits);

	bool validateSickBed(Sickbed &sickbed,SickbedLimits &limits);

	void validateCluster(std::vector<int> &clusters,unsigned int &num_of_clusters, int &count, unsigned int prev_idx);

	void registration();

	void processLaserMsg();

	void makeSickbedFootprint();

	bool createJsonString(Sickbed &object, Json::Value &json);

	bool createJsonString(std::vector<Sickbed> &objectslist, std::string &json_string);

	boost::mutex laser_mtx;

	std::vector<sensor_msgs::LaserScan> laser_msg_buffer;
	tf::TransformListener *tf_;

	ros::Subscriber laser_sub_;
	ros::Publisher objects_pub_,  objects_footprint_pub_,node_pub;
	std::list<LegPaar>legpaar_width_candidates,legpaar_length_candidates;
	std::vector<Sickbed> sickbedlist;
	std::vector<geometry_msgs::Point > sickbed_footprint_;
	SickbedLimits sickbedlimits_,smallsickbedlimits_;
	double cluster_threshold_,max_detect_threshold_;
	int num_of_ptr_threshold_;
	bool should_filter_;
};

};




#endif /* NAVIGATION_LAYERS_OBJECTS_DETECTOR_INCLUDE_OBJECTS_DETECTOR_OBJECTS_DETECTOR_H_ */


