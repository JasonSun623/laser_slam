/*
 * people_detector.cpp
 *
 *  Created on: Feb 7, 2018
 *      Author: tomzhang
 */
#include "objects_detector/people_detector.h"

#include <filters/filter_base.h>

#include <pluginlib/class_list_macros.h>


PLUGINLIB_DECLARE_CLASS(objects_detector, PeopleDetector, objects_detector::PeopleDetector, filters::FilterBase<sensor_msgs::LaserScan>)
using namespace objects_detector;

//int main(int argc, char **argv){
//
//	ros::init(argc, argv, "edge_leg_detector");
//	tf::TransformListener tf(ros::Duration(10));
//	ros::NodeHandle n;
//	ros::Publisher  node_pub = n.advertise <geometry_msgs::PoseArray>("edge_leg_detector", 2); // Humans in the environment
//	ros::Publisher event_pub = n.advertise<std_msgs::String>("/app_custom_event_sub", 10);
//	ros::Publisher leg_pub =n.advertise<std_msgs::String>("/objects", 10);
//	//last_report_time = ros::Time::now();
//
//	// get param from launch file
//	string laser_scan = "/scan";
//	ros::param::get("~laser_scan", laser_scan);
//	ros::Subscriber node_sub = n.subscribe(laser_scan, 2, LaserCallback);
//
//	n.param("map_frame_id", map_frame_id, string("map"));
//	//n.param("detect_xy_range", detect_xy_range, 5.0);
//	// n.param("detect_angle_range", detect_angle_range, 50.0);
//
//	geometry_msgs::PoseArray msgx;
//	ros::Rate loop_rate(15);
//	int seq_counter = 0;
//	int out_range = 0;
//	while( ros::ok() )
//	{
//		if( sensor_on == true )
//		{
//			// delete persons who are too near to each other
//			boost::mutex::scoped_lock lock(mutex);
//			void ValidateDistance();
//
//			//------------------------------------------
//			// Copying to proper PoseArray data structure
//			vector < geometry_msgs::Pose > HumanPoseVector;
//			//Json::Value data;
//			int person_in_range = 0;
//			for( int K = 0; K < rec_x.size(); K++ )
//			{
//				geometry_msgs::Point HumanPoint;
//				geometry_msgs::Quaternion HumanQuaternion;
//
//				HumanPoint.x = rec_x[ K ];
//				HumanPoint.y = rec_y[ K ];
//				HumanPoint.z = 0;
//
//				HumanQuaternion.x = 0;//|-> Orientation is ignored
//				HumanQuaternion.y = 0;//|
//				HumanQuaternion.z = 0;//|
//				HumanQuaternion.w = 1;//|
//
//				geometry_msgs::Pose HumanPose;
//				HumanPose.position = HumanPoint;
//				HumanPose.orientation= HumanQuaternion;
//				HumanPoseVector.push_back( HumanPose );
//				//        double angle = atan2(rec_x[K],rec_y[K]);
//				//        double dist = sqrt(rec_x[K]*rec_x[K]+rec_y[K]*rec_y[K]);
//				//        double angle_limit = detect_angle_range*M_PI/180.0;
//				//        ROS_INFO("angle:%f,dist:%f,detect_xy_range:%f,detect_xy_range:%f,anglelimit:%f",angle,dist,detect_angle_range,detect_xy_range,angle_limit);
//				//        if((fabs(angle) < angle_limit) && (dist < detect_xy_range))
//				//        {
//				//            data[person_in_range]["person_x"] = rec_x[K];
//				//            data[person_in_range]["person_y"] = rec_y[K];
//				//            person_in_range++;
//				//        }
//
//			}
//			//      ROS_INFO("person in range:%d",person_in_range);
//
//			// Header config
//			msgx.header.stamp =last_report_time;//ros::Time::now();
//			msgx.header.frame_id = SensorMsg.header.frame_id;
//			msgx.header.seq = seq_counter;
//			msgx.poses = HumanPoseVector;
//			//------------------------------------------
//			node_pub.publish( msgx );
//			lock.unlock();
//			if(!HumanPoseVector.empty())
//			{
//				if(!tf.waitForTransform(map_frame_id,msgx.header.frame_id,
//						msgx.header.stamp, ros::Duration(0.1)))
//				{
//					ROS_ERROR_THROTTLE(1.0, "leg detector can't transform from %s to %s at %f",
//							map_frame_id.c_str(), msgx.header.frame_id.c_str(),
//							msgx.header.stamp.toSec());
//				}
//				else
//				{
//					Json::Value objects;
//					geometry_msgs::PoseStamped pose_in,pose_out;
//					pose_in.header = msgx.header;
//					for(int i = 0; i<HumanPoseVector.size();++i)
//					{
//						pose_in.pose = HumanPoseVector[i];
//						try
//						{
//							tf.transformPose(map_frame_id,pose_in,pose_out);
//							Json::Value object;
//							object["type"] = 0;
//							object["duration"] = 0.1;
//							object["stamp"] = pose_out.header.stamp.toSec();
//							object["frame_id"] = pose_out.header.frame_id;
//							object["center"].append(Json::Value(pose_out.pose.position.x));
//							object["center"].append(Json::Value(pose_out.pose.position.y));
//							object["center"].append(Json::Value(pose_out.pose.position.z));
//							object["radius"] = 0.3;
//							objects["objects"].append(object);
//						}
//						catch (tf::TransformException& ex)
//						{
//							ROS_ERROR("TF Error attempting to transform an HumanPoseVector from %s to %s: %s", map_frame_id.c_str(),
//									pose_in.header.frame_id.c_str(), ex.what());
//						}
//					}
//					Json::StyledWriter fast;
//					std_msgs::String str;
//					str.data = fast.write(objects);
//					leg_pub.publish(str);
//				}
//				//
//				//      if(0 != person_in_range)
//				//      {
//				//          //if((ros::Time::now() - last_report_time > ros::Duration(1.0)) || (false == last_in_range_flag))
//				//          /*if(false == last_in_range_flag)
//				//          {
//				//              Json::Value msg;
//				//              Json::StyledWriter fast;
//				//              msg["event_type"] = 1;
//				//              msg["event_data"] = data;
//				//              std_msgs::String str;
//				//              str.data = fast.write(msg);
//				//              event_pub.publish(str);
//				//              last_report_time = ros::Time::now();
//				//              ROS_INFO("report enter event");
//				//          }
//				//          last_in_range_flag = true;*/
//				//          if((false == last_in_range_flag) && (ros::Time::now() - last_report_time > ros::Duration(0.5)))
//				//          {
//				//              Json::Value msg;
//				//              Json::StyledWriter fast;
//				//              msg["event_type"] = 1;
//				//              msg["event_data"] = data;
//				//              std_msgs::String str;
//				//              str.data = fast.write(msg);
//				//              event_pub.publish(str);
//				//              last_report_time = ros::Time::now();
//				//              last_in_range_flag = true;
//				//              ROS_INFO("report enter event");
//				//          }
//				//          out_range = 0;
//				//
//				//      }
//				//      else
//				//      {
//				//          //if((ros::Time::now() - last_report_time > ros::Duration(1.0)) || (last_in_range_flag))
//				//          /*if(last_in_range_flag)
//				//          {
//				//              Json::Value msg;
//				//              Json::StyledWriter fast;
//				//              msg["event_type"] = 2;
//				//              msg["event_data"] = "null";
//				//              std_msgs::String str;
//				//              str.data = fast.write(msg);
//				//              event_pub.publish(str);
//				//              last_report_time = ros::Time::now();
//				//              ROS_INFO("report exit event");
//				//          }
//				//          last_in_range_flag = false;*/
//				//
//				//          if(last_in_range_flag && (ros::Time::now() - last_report_time > ros::Duration(0.5)) && (out_range > 5))
//				//          {
//				//              Json::Value msg;
//				//              Json::StyledWriter fast;
//				//              msg["event_type"] = 2;
//				//              msg["event_data"] = "null";
//				//              std_msgs::String str;
//				//              str.data = fast.write(msg);
//				//              event_pub.publish(str);
//				//              last_report_time = ros::Time::now();
//				//              last_in_range_flag = false;
//				//              ROS_INFO("report exit event");
//				//          }
//				//          out_range++;
//				//      }
//				//      ROS_INFO("out_range:%d",out_range);
//			}
//		}
//		ros::spinOnce();
//		loop_rate.sleep();
//		seq_counter++;
//	}
//
//	return 0;
//}

// Euclidean distance between two coordinate points
inline double objects_detector::Dist2D( double x0, double y0, double x1, double y1 ){
	return sqrt( pow( x0 - x1, 2 ) + pow( y0 - y1, 2 ) );
}
PeopleDetector::PeopleDetector()
{

}
PeopleDetector::~PeopleDetector()
{
	if(tf_)
		delete tf_;
}
bool PeopleDetector::configure()
{
	tf_ = new tf::TransformListener(ros::Duration(10));
	sensor_on   = false;

	g_counter = 0;

	last_in_range_flag = false;
	if(!getParam("filter",should_filter_))
	{
		should_filter_ = false;
	}

	detect_xy_range = 5.0;
	detect_angle_range = 90.0;

	seq_counter = 0;
	ros::NodeHandle n;
	n.param("map_frame_id", map_frame_id, string("map"));

	node_pub = n.advertise <geometry_msgs::PoseArray>("edge_leg_detector", 2); // Humans in the environment
	event_pub = n.advertise<std_msgs::String>("/app_custom_event_sub", 10);
	leg_pub = n.advertise<std_msgs::String>("/objects", 10);
	return true;
}
bool PeopleDetector::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
	rec_x.clear();
	rec_y.clear();
	scan_out = scan_in;
	double px, py, pr, pt;
	vector < double >  laser_x;
	vector < double >  laser_y;
	vector < double >  laser_r;
	vector < double >  laser_t;
	for( unsigned i = 0; i < scan_out.ranges.size(); ++i ){
		pr = scan_out.ranges[ i ];
		pt = scan_out.angle_min + ( i * scan_out.angle_increment);
		laser_r.push_back( pr );
		laser_t.push_back( pt );
	}

	// Filtering laser scan
	if(should_filter_)
		LaserFilter_Mean( &laser_r, FILTER_SIZE );
	for( unsigned i = 0; i < scan_out.ranges.size(); ++i ){
		px = laser_r[ i ] * cos( laser_t[ i ] );
		py = laser_r[ i ] * sin( laser_t[ i ] );
		laser_x.push_back( px );
		laser_y.push_back( py );
	}

	string str_aux = "";
	// Finding flanks in the laser scan...
	vector < int > laser_flank;
	laser_flank.assign(laser_r.size(), 0);
	for( unsigned i = 1; i < laser_flank.size(); ++i ){
		if( fabs( laser_r[ i ] - laser_r[ i - 1 ] ) > FLANK_THRESHOLD )
			laser_flank[ i ] = ( ( laser_r[ i ] - laser_r[ i - 1 ] ) > 0 ) ? FLANK_U : FLANK_D;
	}

	vector < int >  flank_id0;
	vector < int >  flank_id1;
	string flank_string = "";
	int past_value = 0;
	int idx = 0;
	for( unsigned i = 1; i < laser_flank.size(); ++i )
	{
		if( laser_flank[ i ] != 0 )
		{
			if( past_value != laser_flank[ i ] )
			{
				flank_id0.push_back( i - 1 );
				flank_id1.push_back( i );
				flank_string += ( laser_flank[ i ] > 0 ) ? "S" : "B";
				idx++;
			}
			else
				flank_id1[ idx - 1 ] =  i;
		}
		past_value = laser_flank[ i ];
	}

	// PATTERN RECOGNITION
	string LEGS_LA  = "BSBS";
	string LEGS_FS1 = "BBS";
	string LEGS_FS2 = "BSS";
	string LEGS_SL = "BS";

	list <int> Pattern_LA;
	list <int> Pattern_FS1;
	list <int> Pattern_FS2;
	list <int> Pattern_SL;

	//ROS_INFO("flank string:%s",flank_string.c_str());
	FindPattern( flank_string, LEGS_LA,  &Pattern_LA  );
	FindPattern( flank_string, LEGS_FS1, &Pattern_FS1 );
	FindPattern( flank_string, LEGS_FS2, &Pattern_FS2 );
	FindPattern( flank_string, LEGS_SL,  &Pattern_SL  );
	//ROS_INFO("LA:%d,FS1:%d,FS2:%d,SL:%d",Pattern_LA.size(),Pattern_FS1.size(),Pattern_FS2.size(),Pattern_SL.size());

	// ANTROPOMETRIC VALIDATION (the non antropometric patterns are erased from the list)
	//judge legs using distance of legs.
	ValidatePattern( &Pattern_LA,  TYPE_LA, flank_id0, flank_id1,  laser_x, laser_y);
	ValidatePattern( &Pattern_FS1, TYPE_FS, flank_id0, flank_id1,  laser_x, laser_y);
	ValidatePattern( &Pattern_FS2, TYPE_FS, flank_id0, flank_id1,  laser_x, laser_y);
	ValidatePattern( &Pattern_SL,  TYPE_SL, flank_id0, flank_id1,  laser_x, laser_y);

	// ERASE REDUNDANT PATTERNS FROM ACCEPTED ONES (If a LA or FS pattern is accepted, we erase the SL on it)
	// a) Erase SL from LA
	list<int>::iterator it_K;
	for( it_K = Pattern_LA.begin(); it_K != Pattern_LA.end(); ++it_K )
	{
		list<int>::iterator it_M;
		// Erase first leg
		for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); ++it_M )
			if( flank_id0[ *it_K ] == flank_id0[ *it_M ] )
			{
				Pattern_SL.erase( it_M );
				break;
			}
		// Erase second leg
		for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); ++it_M )
			if( flank_id0[ *it_K + 2 ] == flank_id0[ *it_M ] )
			{
				Pattern_SL.erase( it_M );
				break;
			}

	}
	// b) Erase SL from FS1 "BBS"
	for( it_K = Pattern_FS1.begin(); it_K != Pattern_FS1.end(); ++it_K )
	{
		list<int>::iterator it_M;
		for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); ++it_M )
			if( flank_id0[ *it_K + 1 ] == flank_id0[ *it_M ] )
			{
				Pattern_SL.erase( it_M );
				break;
			}
	}
	// c) Erase SL from FS2 "BSS"
	for( it_K = Pattern_FS1.begin(); it_K != Pattern_FS1.end(); ++it_K )
	{
		list<int>::iterator it_M;
		for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); ++it_M )
			if( flank_id0[ *it_K ] == flank_id0[ *it_M ] )
			{
				Pattern_SL.erase( it_M );
				break;
			}
	}

	//CENTROID PATTERN COMPUTATION & UNCERTAINTY


	HumanPose( &rec_x, &rec_y, Pattern_LA,  TYPE_LA,  flank_id0, flank_id1,  laser_x, laser_y);
	HumanPose( &rec_x, &rec_y, Pattern_FS1, TYPE_FS,  flank_id0, flank_id1,  laser_x, laser_y);
	HumanPose( &rec_x, &rec_y, Pattern_FS2, TYPE_FS,  flank_id0, flank_id1,  laser_x, laser_y);
	HumanPose( &rec_x, &rec_y, Pattern_SL,  TYPE_SL,  flank_id0, flank_id1,  laser_x, laser_y);

	void ValidateDistance();

	//------------------------------------------
	// Copying to proper PoseArray data structure
	vector < geometry_msgs::Pose > HumanPoseVector;
	//Json::Value data;
	int person_in_range = 0;
	for( int K = 0; K < rec_x.size(); ++K )
	{
		geometry_msgs::Point HumanPoint;
		geometry_msgs::Quaternion HumanQuaternion;

		HumanPoint.x = rec_x[ K ];
		HumanPoint.y = rec_y[ K ];
		HumanPoint.z = 0;

		HumanQuaternion.x = 0;//|-> Orientation is ignored
		HumanQuaternion.y = 0;//|
		HumanQuaternion.z = 0;//|
		HumanQuaternion.w = 1;//|

		geometry_msgs::Pose HumanPose;
		HumanPose.position = HumanPoint;
		HumanPose.orientation= HumanQuaternion;
		HumanPoseVector.push_back( HumanPose );
//		double angle = atan2(rec_x[K],rec_y[K]);
//		double dist = sqrt(rec_x[K]*rec_x[K]+rec_y[K]*rec_y[K]);
//		double angle_limit = detect_angle_range*M_PI/180.0;
//		ROS_INFO("angle:%f,dist:%f,detect_xy_range:%f,detect_xy_range:%f,anglelimit:%f",angle,dist,detect_angle_range,detect_xy_range,angle_limit);
//		if((fabs(angle) < angle_limit) && (dist < detect_xy_range))
//		{
//			data[person_in_range]["person_x"] = rec_x[K];
//			data[person_in_range]["person_y"] = rec_y[K];
//			person_in_range++;
//		}

	}
//	ROS_INFO("person in range:%d",person_in_range);
//
//	Header config
	geometry_msgs::PoseArray msgx;
	msgx.header.stamp =scan_out.header.stamp;//ros::Time::now();
	msgx.header.frame_id = scan_out.header.frame_id;
	msgx.header.seq = seq_counter;
	msgx.poses = HumanPoseVector;
	//------------------------------------------
	node_pub.publish( msgx );

	if(!HumanPoseVector.empty())
	{
		if(!tf_->waitForTransform(map_frame_id,msgx.header.frame_id,
				msgx.header.stamp, ros::Duration(0.1)))
		{
			ROS_ERROR_THROTTLE(1.0, "leg detector can't transform from %s to %s at %f",
					map_frame_id.c_str(), msgx.header.frame_id.c_str(),
					msgx.header.stamp.toSec());
		}
		else
		{
			Json::Value objects;
			geometry_msgs::PoseStamped pose_in,pose_out;
			pose_in.header = msgx.header;
			for(int i = 0; i<HumanPoseVector.size();++i)
			{
				pose_in.pose = HumanPoseVector[i];
				try
				{
					tf_->transformPose(map_frame_id,pose_in,pose_out);
					Json::Value object;
					object["type"] = 0;
					object["duration"] = 0.1;
					object["stamp"] = pose_out.header.stamp.toSec();
					object["frame_id"] = pose_out.header.frame_id;
					object["center"].append(Json::Value(pose_out.pose.position.x));
					object["center"].append(Json::Value(pose_out.pose.position.y));
					object["center"].append(Json::Value(pose_out.pose.position.z));
					object["radius"] = 0.3;
					objects["objects"].append(object);
				}
				catch (tf::TransformException& ex)
				{
					ROS_ERROR("TF Error attempting to transform an HumanPoseVector from %s to %s: %s", map_frame_id.c_str(),
							pose_in.header.frame_id.c_str(), ex.what());
				}
			}
			Json::StyledWriter fast;
			std_msgs::String str;
			str.data = fast.write(objects);
			leg_pub.publish(str);
		}
	}
	return true;
}

//	void PeopleDetector::LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg){
//
//		// To get header data from sensor msg
//		SensorMsg = *msg;
//
//		// Vectors...
//		rec_x.clear();
//		rec_y.clear();
//
//		sensor_on = true;
//
//		double px, py, pr, pt;
//		vector < double >  laser_x;
//		vector < double >  laser_y;
//		vector < double >  laser_r;
//		vector < double >  laser_t;
//		for( unsigned i = 0; i < msg->ranges.size(); i++ ){
//			pr = msg->ranges[ i ];
//			pt = msg->angle_min + ( i * msg->angle_increment);
//			laser_r.push_back( pr );
//			laser_t.push_back( pt );
//		}
//
//		// Filtering laser scan
//		LaserFilter_Mean( &laser_r, FILTER_SIZE );
//		for( unsigned i = 0; i < msg->ranges.size(); i++ ){
//			px = laser_r[ i ] * cos( laser_t[ i ] );
//			py = laser_r[ i ] * sin( laser_t[ i ] );
//			laser_x.push_back( px );
//			laser_y.push_back( py );
//		}
//
//		string str_aux = "";
//		// Finding flanks in the laser scan...
//		vector < int > laser_flank;
//		laser_flank.assign(laser_r.size(), 0);
//		for( unsigned i = 1; i < laser_flank.size(); i++ ){
//			if( fabs( laser_r[ i ] - laser_r[ i - 1 ] ) > FLANK_THRESHOLD )
//				laser_flank[ i ] = ( ( laser_r[ i ] - laser_r[ i - 1 ] ) > 0 ) ? FLANK_U : FLANK_D;
//		}
//
//		vector < int >  flank_id0;
//		vector < int >  flank_id1;
//		string flank_string = "";
//		int past_value = 0;
//		int idx = 0;
//		for( unsigned i = 1; i < laser_flank.size(); i++ )
//		{
//			if( laser_flank[ i ] != 0 )
//			{
//				if( past_value != laser_flank[ i ] )
//				{
//					flank_id0.push_back( i - 1 );
//					flank_id1.push_back( i );
//					flank_string += ( laser_flank[ i ] > 0 ) ? "S" : "B";
//					idx++;
//				}
//				else
//					flank_id1[ idx - 1 ] =  i;
//			}
//			past_value = laser_flank[ i ];
//		}
//
//		// PATTERN RECOGNITION
//		string LEGS_LA  = "BSBS";
//		string LEGS_FS1 = "BBS";
//		string LEGS_FS2 = "BSS";
//		string LEGS_SL = "BS";
//
//		list <int> Pattern_LA;
//		list <int> Pattern_FS1;
//		list <int> Pattern_FS2;
//		list <int> Pattern_SL;
//
//		//ROS_INFO("flank string:%s",flank_string.c_str());
//		FindPattern( flank_string, LEGS_LA,  &Pattern_LA  );
//		FindPattern( flank_string, LEGS_FS1, &Pattern_FS1 );
//		FindPattern( flank_string, LEGS_FS2, &Pattern_FS2 );
//		FindPattern( flank_string, LEGS_SL,  &Pattern_SL  );
//		//ROS_INFO("LA:%d,FS1:%d,FS2:%d,SL:%d",Pattern_LA.size(),Pattern_FS1.size(),Pattern_FS2.size(),Pattern_SL.size());
//
//		// ANTROPOMETRIC VALIDATION (the non antropometric patterns are erased from the list)
//		//judge legs using distance of legs.
//		ValidatePattern( &Pattern_LA,  TYPE_LA, flank_id0, flank_id1,  laser_x, laser_y);
//		ValidatePattern( &Pattern_FS1, TYPE_FS, flank_id0, flank_id1,  laser_x, laser_y);
//		ValidatePattern( &Pattern_FS2, TYPE_FS, flank_id0, flank_id1,  laser_x, laser_y);
//		ValidatePattern( &Pattern_SL,  TYPE_SL, flank_id0, flank_id1,  laser_x, laser_y);
//
//		// ERASE REDUNDANT PATTERNS FROM ACCEPTED ONES (If a LA or FS pattern is accepted, we erase the SL on it)
//		// a) Erase SL from LA
//		list<int>::iterator it_K;
//		for( it_K = Pattern_LA.begin(); it_K != Pattern_LA.end(); it_K++ )
//		{
//			list<int>::iterator it_M;
//			// Erase first leg
//			for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++ )
//				if( flank_id0[ *it_K ] == flank_id0[ *it_M ] )
//				{
//					Pattern_SL.erase( it_M );
//					break;
//				}
//			// Erase second leg
//			for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++ )
//				if( flank_id0[ *it_K + 2 ] == flank_id0[ *it_M ] )
//				{
//					Pattern_SL.erase( it_M );
//					break;
//				}
//
//		}
//		// b) Erase SL from FS1 "BBS"
//		for( it_K = Pattern_FS1.begin(); it_K != Pattern_FS1.end(); it_K++ )
//		{
//			list<int>::iterator it_M;
//			for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++ )
//				if( flank_id0[ *it_K + 1 ] == flank_id0[ *it_M ] )
//				{
//					Pattern_SL.erase( it_M );
//					break;
//				}
//		}
//		// c) Erase SL from FS2 "BSS"
//		for( it_K = Pattern_FS1.begin(); it_K != Pattern_FS1.end(); it_K++ )
//		{
//			list<int>::iterator it_M;
//			for( it_M = Pattern_SL.begin(); it_M != Pattern_SL.end(); it_M++ )
//				if( flank_id0[ *it_K ] == flank_id0[ *it_M ] )
//				{
//					Pattern_SL.erase( it_M );
//					break;
//				}
//		}
//
//
//		boost::mutex::scoped_lock lock(mutex);
//		//CENTROID PATTERN COMPUTATION & UNCERTAINTY
//		rec_x.clear();
//		rec_y.clear();
//
//		HumanPose( &rec_x, &rec_y, Pattern_LA,  TYPE_LA,  flank_id0, flank_id1,  laser_x, laser_y);
//		HumanPose( &rec_x, &rec_y, Pattern_FS1, TYPE_FS,  flank_id0, flank_id1,  laser_x, laser_y);
//		HumanPose( &rec_x, &rec_y, Pattern_FS2, TYPE_FS,  flank_id0, flank_id1,  laser_x, laser_y);
//		HumanPose( &rec_x, &rec_y, Pattern_SL,  TYPE_SL,  flank_id0, flank_id1,  laser_x, laser_y);
//
//		last_report_time = SensorMsg.header.stamp;
//	}
//
//
	// Mean value of the 'size' adjacent values
	void PeopleDetector::LaserFilter_Mean( vector <double> *vector_r, unsigned size )
	{
		for( unsigned i = 0; i < ( (*vector_r).size() - size ); i++ )
		{
			double mean = 0;
			for( unsigned k = 0; k < size; k++  )
			{
				mean += (*vector_r)[ i + k ];
			}
			(*vector_r)[ i ] = mean / size;
		}
	}


	// Reports a found string pattern in a list
	void PeopleDetector::FindPattern( string str, string pattern, list <int> *element_found )
	{
		size_t found = 0;

		while( string::npos != ( found = str.find( pattern, found ) ) )
		{
			(*element_found).push_back( found );
			found++;
		}

	}

	/*
	 * #define ANTRO_a0 0.1 //|
#define ANTRO_a1 0.2 //|-> Leg width (min-max)
#define ANTRO_b0 0   //  |
#define ANTRO_b1 0.4 //  |-> Free space between two legs (min-max)
#define ANTRO_c0 0.1 //    |
#define ANTRO_c1 0.4 //    |-> Two legs together width (min-max)
	 */
	// Performs the antropometric validation of the leg patterns
	void PeopleDetector::ValidatePattern( list <int> *Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y){

		double ANTRO_a_1, ANTRO_a_2, ANTRO_b, ANTRO_c; // Antropometric values from patterns to compare with constants.
		bool SavePattern = true;
		bool cond_a = true, cond_b = true, cond_c = true;
		list<int>::iterator it;

		for( it = (*Pattern_list).begin(); it != (*Pattern_list).end(); it++ )
		{
			// Obtain antropometric values
			switch( TYPE )
			{
			case TYPE_LA: //BSBS
				ANTRO_a_1 = Dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
				ANTRO_a_2 = Dist2D( laser_x[ flank_id1[ *it + 2 ] ], laser_y[ flank_id1[ *it + 2 ] ], laser_x[ flank_id0[ *it + 3 ] ], laser_y[ flank_id0[ *it + 3 ] ]);
				ANTRO_b = Dist2D( laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ], laser_x[ flank_id1[ *it + 2 ] ], laser_y[ flank_id1[ *it + 2 ] ] );
				ANTRO_c = 0;
				cond_a = ( ( ANTRO_a_1 >= ANTRO_a0 ) && ( ANTRO_a_1 <= ANTRO_a1 ) ) && ( ( ANTRO_a_2 >= ANTRO_a0 ) && ( ANTRO_a_2 <= ANTRO_a1 ) );
				cond_b = ( ( ANTRO_b >= ANTRO_b0 ) && ( ANTRO_b <= ANTRO_b1 ) );
				cond_c = true;
				break;
			case TYPE_FS: // BBS & BSS
				ANTRO_a_1 = Dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
				ANTRO_a_2 = Dist2D( laser_x[ flank_id1[ *it + 1 ] ], laser_y[ flank_id1[ *it + 1 ] ], laser_x[ flank_id0[ *it + 2 ] ], laser_y[ flank_id0[ *it + 2 ] ]);
				ANTRO_b = Dist2D( laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ], laser_x[ flank_id1[ *it + 1 ] ], laser_y[ flank_id1[ *it + 1 ] ] );
				ANTRO_c = 0;
				cond_a = ( ( ANTRO_a_1 >= ANTRO_a0 ) && ( ANTRO_a_1 <= ANTRO_a1 ) ) && ( ( ANTRO_a_2 >= ANTRO_a0 ) && ( ANTRO_a_2 <= ANTRO_a1 ) );
				cond_b = ( ( ANTRO_b >= ANTRO_b0 ) && ( ANTRO_b <= ANTRO_b1 ) );
				cond_c = true;
				break;
			case TYPE_SL: // BS
				ANTRO_a_1 = 0;
				ANTRO_a_2 = 0;
				ANTRO_b = 0;
				ANTRO_c = Dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
				cond_a = true;
				cond_b = true;
				cond_c = ( ( ANTRO_c >= ANTRO_c0 ) && ( ANTRO_c <= ANTRO_c1 ) );
				break;
			}

			SavePattern = cond_a && cond_b && cond_c;

			if( !SavePattern )
			{
				it = (*Pattern_list).erase( it );
				it--;
			}
		}
	}





	void PeopleDetector::HumanPose( vector <double> *r_x, vector <double> *r_y, list <int> Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y ){

		double c_x, c_y;
		int l1, l2, l3, l4;
		int count;
		list<int>::iterator it;

		for( it = Pattern_list.begin(); it != Pattern_list.end(); it++ )
		{
			c_x = 0;
			c_y = 0;
			count = 0;

			l1 = flank_id1[ *it ];
			l2 = flank_id0[ *it + 1 ];

			switch( TYPE )
			{
			case TYPE_LA:
				l3 = flank_id1[ *it + 2 ];
				l4 = flank_id0[ *it + 3 ];
				break;
			case TYPE_FS:
				l3 = flank_id1[ *it + 1 ];
				l4 = flank_id0[ *it + 2 ];
				break;
			case TYPE_SL:
				l3 = 1;
				l4 = 0;
				break;
			}

			for( int i = l1; i <= l2; i++ )
			{
				c_x += laser_x[ i ];
				c_y += laser_y[ i ];
				count++;
			}
			for( int i = l3; i <= l4; i++ )
			{
				c_x += laser_x[ i ];
				c_y += laser_y[ i ];
				count++;
			}

			c_x /= (double) count;
			c_y /= (double) count;

			(*r_x).push_back( c_x );
			(*r_y).push_back( c_y );
		}
	}


	// Validate distance between persons
	void PeopleDetector::ValidateDistance()
	{
		int j = 0;
		while(j < (rec_x.size() - 1))
		{
			// if the Euclidean distance between two persons are smaller than
			// the maximum width of a leg then the second person must be eliminated
			if (ANTRO_b1 > Dist2D(rec_x[j], rec_y[j], rec_x[j+1], rec_y[j+1]))
			{
				rec_x.erase(rec_x.begin() + (j + 1));
				rec_y.erase(rec_y.begin() + (j + 1));
			}
			else
			{
				j++;
			}
		}
	}


