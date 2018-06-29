/*
 * leg_detector.h
 *
 *  Created on: Feb 7, 2018
 *      Author: tomzhang
 */

#ifndef NAVIGATION_LAYERS_OBJECTS_DETECTOR_INCLUDE_OBJECTS_DETECTOR_PEOPLE_DETECTOR_H_
#define NAVIGATION_LAYERS_OBJECTS_DETECTOR_INCLUDE_OBJECTS_DETECTOR_PEOPLE_DETECTOR_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <boost/thread/mutex.hpp>
#include <filters/filter_base.h>
#include "json/json.h"
#include "std_msgs/String.h"

#define PI 3.1416
#define FILTER_SIZE 2
#define FLANK_THRESHOLD 0.1

#define FLANK_U 1
#define FLANK_D -1

//Antropometric parameters
#define ANTRO_a0 0.1 //|
#define ANTRO_a1 0.2 //|-> Leg width (min-max)
#define ANTRO_b0 0   //  |
#define ANTRO_b1 0.4 //  |-> Free space between two legs (min-max)
#define ANTRO_c0 0.1 //    |
#define ANTRO_c1 0.4 //    |-> Two legs together width (min-max)

// Pattern Type
#define TYPE_LA 1 // Legs separated
#define TYPE_FS 2 // Legs dephased
#define TYPE_SL 3 // Legs together
using namespace std;
namespace objects_detector
{
double Dist2D( double x0, double y0, double x1, double y1 );
class PeopleDetector:public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
	PeopleDetector();
	~PeopleDetector();
	bool configure();
    bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);

private:

	void LaserFilter_Mean( vector <double> *vector_r, unsigned size );
	//void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg);
	void FindPattern( string str, string pattern, list <int> *element_found );
	void ValidatePattern( list <int> *Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y);

	void HumanPose( vector <double> *r_x, vector <double> *r_y, list <int> Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y );

	// added by Ferdian Jovan
	// function to restrict a possibility of persons standing next to each other
	void ValidateDistance();

	bool sensor_on;

	int g_counter;

	vector < double > rec_x;
	vector < double > rec_y;
	string sensor_frame_id;
	string map_frame_id;
	sensor_msgs::LaserScan SensorMsg;
	boost::mutex mutex;
	bool last_in_range_flag,should_filter_;

	double detect_xy_range;
	double detect_angle_range;
	ros::Time last_report_time;

	ros::Publisher  node_pub;
	ros::Publisher event_pub;
	ros::Publisher leg_pub;
	int seq_counter;
	tf::TransformListener *tf_;
};

}


#endif /* NAVIGATION_LAYERS_OBJECTS_DETECTOR_INCLUDE_OBJECTS_DETECTOR_PEOPLE_DETECTOR_H_ */
