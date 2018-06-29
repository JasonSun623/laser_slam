/*
 * objects_detector.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: tomzhang
 */



#include "objects_detector/objects_detector.h"
#include <filters/filter_base.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(objects_detector, ObjectsDetector, objects_detector::ObjectsDetector, filters::FilterBase<sensor_msgs::LaserScan>)
using namespace objects_detector;
#define FILTER_SIZE 2
#define NO_CLUSTER -1


inline bool objects_detector::interval( double number, double lower, double upper )
{
	return (number <= upper && number >=lower);
}
inline double objects_detector::getEulerDistance(tf::Stamped<tf::Pose> &point1, tf::Stamped<tf::Pose> &point2)
{
	return point1.getOrigin().distance(point2.getOrigin());
}
ObjectsDetector::ObjectsDetector()
{




}
ObjectsDetector::~ObjectsDetector()
{

	if(tf_!=NULL)
		delete tf_;


}
bool ObjectsDetector::configure()
{
	ros::NodeHandle nh("~");
	tf_ = new tf::TransformListener();
	laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>("/scan",100,&ObjectsDetector::laserCallback, this);
	objects_pub_ = nh.advertise<std_msgs::String>("/objects",100);
	node_pub = nh.advertise <geometry_msgs::PoseArray>("/objects_detector", 2);
	objects_footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);

	if(!getParam("filter",should_filter_))
	{
		cluster_threshold_ = false;
	}
	if(!getParam("cluster_threshold",cluster_threshold_))
	{
		cluster_threshold_ = 0.05;
	}
	if(!getParam("num_of_ptr_threshold",num_of_ptr_threshold_))
	{
		num_of_ptr_threshold_ = 3;
	}
	if(!getParam("max_detect_threshold",max_detect_threshold_))
	{
		max_detect_threshold_ = 4.0;
	}
	if(!getParam("leg_width_lower",sickbedlimits_.leg_width_lower))
	{
		sickbedlimits_.leg_width_lower=0.15;
	}
	if(!getParam("leg_width_upper",sickbedlimits_.leg_width_upper))
	{
		sickbedlimits_.leg_width_upper=0.35;
	}
	if(!getParam("leggap_length_lower",sickbedlimits_.leggap_length_lower))
	{
		sickbedlimits_.leggap_length_lower=0.95;
	}
	if(!getParam("leggap_length_upper",sickbedlimits_.leggap_length_upper))
	{
		sickbedlimits_.leggap_length_upper=1.25;
	}
	if(!getParam("leggap_width_lower",sickbedlimits_.leggap_width_lower))
	{
		sickbedlimits_.leggap_width_lower=0.20;
	}
	if(!getParam("leggap_width_upper",sickbedlimits_.leggap_width_upper))
	{
		sickbedlimits_.leggap_width_upper=0.40;
	}
	if(!getParam("dot_limit",sickbedlimits_.dot_limit))
	{
		sickbedlimits_.dot_limit = 0.35;
	}
	if(!getParam("check_parallel",sickbedlimits_.check_parallel))
	{
		sickbedlimits_.check_parallel=true;
	}

	makeSickbedFootprint();
	laser_msg_buffer.reserve(200);
	return true;
}

void ObjectsDetector::makeSickbedFootprint()
{
	geometry_msgs::Point point;
	point.x = -0.35,point.y =1.0,point.z = 0.0;
	sickbed_footprint_.push_back(point);
	point.x = 0.35,point.y =1.0,point.z = 0.0;
	sickbed_footprint_.push_back(point);
	point.x = 0.35,point.y =-1.0,point.z = 0.0;
	sickbed_footprint_.push_back(point);
	point.x = -0.35,point.y =-1.0,point.z = 0.0;
	sickbed_footprint_.push_back(point);

}
void ObjectsDetector::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(laser_mtx);
	laser_msg_buffer.push_back(*msg);
}

// Mean value of the 'size' adjacent values
void  ObjectsDetector::meanFilter( sensor_msgs::LaserScan &laser_msg, unsigned size )
{
	for( unsigned i = 0; i < ( laser_msg.ranges.size() - size ); ++i )
	{
		double mean = 0;
		for( unsigned k = 0; k < size; k++  )
		{
			mean += laser_msg.ranges[ i + k ];
		}
		laser_msg.ranges[ i ] = mean / size;
	}
}
bool ObjectsDetector::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
	//ROS_ERROR("detect bed");
	scan_out = scan_in;
	if(should_filter_)
		meanFilter( scan_out,FILTER_SIZE );
	std::vector<int> clusters;
	unsigned int num_of_clusters = 0;
	//ROS_INFO("clustering");
	clustering(clusters,num_of_clusters,scan_out);
	//ROS_INFO("size %d",num_of_clusters);
	if( num_of_clusters < 3 )
	{
		return true;
	}
	//ROS_ERROR("detect bed");
	detectSickBed(clusters,scan_out,sickbedlimits_);
	return true;
}
void ObjectsDetector::detect()
{
	processLaserMsg();

}
void ObjectsDetector::processLaserMsg()
{
	//ROS_INFO("process msg");
	std::vector<sensor_msgs::LaserScan> laser_msg_buffer_copy;
	{
		boost::mutex::scoped_lock lock(laser_mtx);
		laser_msg_buffer_copy = std::vector<sensor_msgs::LaserScan>(laser_msg_buffer);
		laser_msg_buffer.clear();

	}
	//ROS_INFO("process copy %d",int(laser_msg_buffer_copy.size()));
	for(unsigned int i = 0; i < laser_msg_buffer_copy.size(); ++i)
	{
		//ROS_INFO("meanFilter %d",i);
		meanFilter( laser_msg_buffer_copy[i],FILTER_SIZE );
		std::vector<int> clusters;
		unsigned int num_of_clusters = 0;
		//ROS_INFO("clustering");
		clustering(clusters,num_of_clusters,laser_msg_buffer_copy[i]);
		//ROS_INFO("size %d",num_of_clusters);
		if( num_of_clusters < 1 )
		{
			continue;
		}
		//ROS_INFO("detect bed");
		detectSickBed(clusters,laser_msg_buffer_copy[i],sickbedlimits_);
		detectSickBed(clusters,laser_msg_buffer_copy[i],smallsickbedlimits_);
		//ROS_INFO("detect bed finish");



	}
}
// check whether the distance between adjectiv lasers shorter than threshold
// if shorter, the two lasers belong same cluster, we count the number of lasers of the cluster
// else, we the laser belong to a new cluster
void ObjectsDetector::clustering(std::vector<int> &clusters,unsigned int &num_of_clusters,sensor_msgs::LaserScan &laser_msg)
{
	if(laser_msg.ranges.empty())
	{
		return;
	}

	unsigned int size = laser_msg.ranges.size();
	//ROS_INFO("not empty %d",size);
	clusters.resize(size,NO_CLUSTER);
	int count = 0;
	if(laser_msg.ranges[size-1] <= max_detect_threshold_)
	{
		clusters[size-1] = size-1;
		++count;
		//ROS_INFO("last laser valid");
	}

	for(int i = size-2; i >=0 ; --i)
	{
		//ROS_INFO("check laser %d",i);
		if(laser_msg.ranges[i] > max_detect_threshold_)
		{
			//ROS_INFO("laser %d > max_detect_threshold_",i);
			validateCluster(clusters,num_of_clusters,count,i+1);
		}
		else
		{
			//ROS_INFO("laser %d < max_detect_threshold_",i);
			if(std::fabs(laser_msg.ranges[i]-laser_msg.ranges[i+1])<cluster_threshold_ && clusters[i+1] != NO_CLUSTER)
			{
				clusters[i] = clusters[i+1];
			}
			else
			{
				validateCluster(clusters,num_of_clusters,count,i+1);
				clusters[i] = i;
			}
			++count;
		}
	}
}
void ObjectsDetector::validateCluster(std::vector<int> &clusters,unsigned int &num_of_clusters, int &count, unsigned int prev_idx)
{
	if(count < num_of_ptr_threshold_)
	{
		for(unsigned int i = 0; i < count; ++i)
		{
			clusters[prev_idx + i] = NO_CLUSTER;
		}
	}
	else
	{
		++num_of_clusters;
	}
	count = 0;
}
void ObjectsDetector::detectSickBed(std::vector<int> &clusters, sensor_msgs::LaserScan &laser_msg,SickbedLimits &limits)
{
	std::vector<Leg> leg_candidates;
	leg_candidates.reserve(100);
	findSickbedFeet(clusters,laser_msg,leg_candidates,limits);
	//ROS_INFO("findSickbedFeet finish");

	machingRestPaar(leg_candidates,limits);
	//ROS_INFO("machingRestPaar finish");
	geometry_msgs::PoseArray msgx;

	geometry_msgs::PoseStamped pose_tf;
	for(int i = 0; i< leg_candidates.size();++i)
	{
		tf::poseStampedTFToMsg(leg_candidates[i].left,pose_tf);
		msgx.poses.push_back(pose_tf.pose);
		tf::poseStampedTFToMsg(leg_candidates[i].center,pose_tf);
		msgx.poses.push_back(pose_tf.pose);
		tf::poseStampedTFToMsg(leg_candidates[i].right,pose_tf);
		msgx.poses.push_back(pose_tf.pose);
	}
	double x = 0.0, y = 0.0, z= 0.0, th= 0.0;
	for(int i = 0; i< sickbedlist.size();++i)
	{
		geometry_msgs::PolygonStamped oriented_footprint;
		tf::poseStampedTFToMsg(sickbedlist[i].center,pose_tf);
		x = sickbedlist[i].center.getOrigin().getX();
		y = sickbedlist[i].center.getOrigin().getY();
		oriented_footprint.header = pose_tf.header;

		th = tf::getYaw(sickbedlist[i].center.getRotation());
		costmap_2d::transformFootprint(x,y,th,sickbed_footprint_,oriented_footprint);
		objects_footprint_pub_.publish(oriented_footprint);
		msgx.poses.push_back(pose_tf.pose);
		for(int p = 0; p <sickbedlist[i].legs.size();++p )
		{
			if(!sickbedlist[i].legs[p]->is_virtual)
			{
				for(unsigned int l =sickbedlist[i].legs[p]->left_idx;l< sickbedlist[i].legs[p]->right_idx; ++i)
				{
					laser_msg.ranges[l] =std::numeric_limits<float>::quiet_NaN();
				}
			}
		}
	}
	std_msgs::String msg;
	if(createJsonString(sickbedlist,msg.data))
	{

		objects_pub_.publish(msg);
	}
	msgx.header = laser_msg.header;
	node_pub.publish(msgx);
	//ROS_INFO("finded leg_candidates %d , legpaar_width_candidates: %d, legpaar_length_candidates: %d,sickbedlist: %d",int(leg_candidates.size()),int(legpaar_width_candidates.size()),int(legpaar_length_candidates.size()),int(sickbedlist.size()));
	sickbedlist.clear();
	legpaar_width_candidates.clear(),
	legpaar_length_candidates.clear();


}
void ObjectsDetector::machingRestPaar(std::vector<Leg> &leg_candidates,SickbedLimits &limits)
{
	if(!legpaar_width_candidates.empty()&&!legpaar_length_candidates.empty())
	{
		std::list<LegPaar>::iterator it_w = legpaar_width_candidates.begin();
		std::list<LegPaar>::iterator it_l;
		for(;it_w!=legpaar_width_candidates.end();)
		{
			bool remove_w = false;
			//if(it_l == legpaar_length_candidates.end())
			//{
			//	return;
			//}
			//ROS_ERROR("w %d, size l %d ",it_w,legpaar_length_candidates.size());
			for(it_l = legpaar_length_candidates.begin();it_l!=legpaar_length_candidates.end();)
			{
				bool remove = false;
				if(it_w->leg1 == it_l->leg1)
				{
					//ROS_ERROR("0 paar sick bed");
					tf::Stamped<tf::Pose> left,right;
					left.frame_id_ = it_w->center.frame_id_;
					left.stamp_ = it_w->center.stamp_;
					right.frame_id_ = it_w->center.frame_id_;
					right.stamp_ = it_w->center.stamp_;
					left.setIdentity();
					left.setOrigin(it_w->leg2->left.getOrigin()+it_l->leg2->left.getOrigin()-it_w->leg1->left.getOrigin());
					right.setIdentity();
					right.setOrigin(it_w->leg2->right.getOrigin()+it_l->leg2->right.getOrigin()-it_w->leg1->right.getOrigin());
					leg_candidates.push_back(Leg(left,right,0,0,true));
					LegPaar legpaar(it_l->leg2,&(leg_candidates.back()));
					Sickbed sickbed(*it_w,legpaar,0);
					if(validateSickBed(sickbed,limits))
					{
						sickbedlist.push_back(sickbed);
						remove = true;
					}

					//TODO createLastLeg(it_w->leg1,it_w->leg2,it_l->leg2);
					//LegPaar leg(it_l->leg2,leg);
					//ROS_ERROR("0 paar sick bed");

				}
				else if(it_w->leg2 == it_l->leg1)
				{
					//ROS_ERROR("1 paar sick bed");
					tf::Stamped<tf::Pose> left,right;
					left.frame_id_ = it_w->center.frame_id_;
					left.stamp_ = it_w->center.stamp_;

					right.frame_id_ = it_w->center.frame_id_;
					right.stamp_ = it_w->center.stamp_;
					left.setIdentity();
					left.setOrigin(it_w->leg1->left.getOrigin()+it_l->leg2->left.getOrigin()-it_w->leg2->left.getOrigin());
					right.setIdentity();
					right.setOrigin(it_w->leg1->right.getOrigin()+it_l->leg2->right.getOrigin()-it_w->leg2->right.getOrigin());
					leg_candidates.push_back(Leg(left,right,0,0,true));
					LegPaar legpaar(&(leg_candidates.back()),it_l->leg2);
					Sickbed sickbed(*it_w,legpaar,0);
					if(validateSickBed(sickbed,limits))
					{
						sickbedlist.push_back(sickbed);
						remove = true;
					}
				}
				else if(it_w->leg2 == it_l->leg2)
				{
					//ROS_ERROR("2 paar sick bed");
					tf::Stamped<tf::Pose> left,right;
					left.frame_id_ = it_w->center.frame_id_;
					left.stamp_ = it_w->center.stamp_;
					right.frame_id_ = it_w->center.frame_id_;
					right.stamp_ = it_w->center.stamp_;
					left.setIdentity();
					left.setOrigin(it_w->leg1->left.getOrigin()+it_l->leg1->left.getOrigin()-it_w->leg2->left.getOrigin());
					right.setIdentity();
					right.setOrigin(it_w->leg1->right.getOrigin()+it_l->leg1->right.getOrigin()-it_w->leg2->right.getOrigin());
					leg_candidates.push_back(Leg(left,right,0,0,true));
					LegPaar legpaar(&(leg_candidates.back()),it_l->leg1);
					Sickbed sickbed(*it_w,legpaar,0);
					if(validateSickBed(sickbed,limits))
					{
						sickbedlist.push_back(sickbed);
						remove = true;
					}

				}
				else if(it_w->leg1 == it_l->leg2)
				{
					//ROS_ERROR("3 paar sick bed0");
					tf::Stamped<tf::Pose> left,right;
					left.frame_id_ = it_w->center.frame_id_;
					left.stamp_ = it_w->center.stamp_;
					right.frame_id_ = it_w->center.frame_id_;
					right.stamp_ = it_w->center.stamp_;
					//ROS_ERROR("%s",left.frame_id_.c_str());
					//ROS_ERROR("%f",left.stamp_.toSec());
					left.setIdentity();
					left.setOrigin(it_w->leg2->left.getOrigin()+it_l->leg1->left.getOrigin()-it_w->leg1->left.getOrigin());
					right.setIdentity();
					right.setOrigin(it_w->leg2->right.getOrigin()+it_l->leg1->right.getOrigin()-it_w->leg1->right.getOrigin());

					//ROS_ERROR("3 paar sick bed1");

					leg_candidates.push_back(Leg(left,right,0,0,true));
					//ROS_ERROR("3 paar sick bed2");
					LegPaar legpaar(it_l->leg1,&(leg_candidates.back()));
					Sickbed sickbed(*it_w,legpaar,0);
					if(validateSickBed(sickbed,limits))
					{
						sickbedlist.push_back(sickbed);
						remove = true;
					}
				}

				if(remove)
				{
					it_w=legpaar_width_candidates.erase(it_w);

					if(it_w == legpaar_width_candidates.end())
					{
						//ROS_ERROR(" == end");
						return;
					}
					it_l=legpaar_length_candidates.erase(it_l);
					//ROS_ERROR("has erase");
					remove_w = true;
				}
				else
				{
					++it_l;
					//ROS_ERROR("++it_l %d",it_l==legpaar_length_candidates.end());
				}
				//std::cout << &(*it_w) << ","<<&(*legpaar_width_candidates.end()) ;
				//ROS_ERROR("%d,%d",it_w,legpaar_width_candidates.end());
				//ROS_ERROR("%d,%d",it_l,legpaar_length_candidates.end());
			}
			if(!remove_w)
			{

				++it_w;
				//ROS_ERROR("++it_w %d",it_w == legpaar_width_candidates.end());
			}

		}
	}
}


void ObjectsDetector::findSickbedFeet(std::vector<int> &clusters, sensor_msgs::LaserScan &laser_msg, std::vector<Leg> &leg_candidates,SickbedLimits &limits)
{
	for(unsigned int i = 0; i < clusters.size(); ++i )
	{
		if(clusters[i] == NO_CLUSTER)
		{
			continue;
		}
		unsigned int left = i, right = clusters[i];

		double r_left =	laser_msg.ranges[left];
		double t_left = laser_msg.angle_min+ left*laser_msg.angle_increment;

		double r_right =	laser_msg.ranges[right];
		double t_right  = laser_msg.angle_min+ right *laser_msg.angle_increment;
		tf::Stamped<tf::Pose> point_left;
		point_left.frame_id_ = laser_msg.header.frame_id;
		point_left.stamp_ = laser_msg.header.stamp;
		point_left.setIdentity();
		point_left.setOrigin(tf::Vector3( r_left * cos( t_left ),r_left * sin( t_left ), 0));

		tf::Stamped<tf::Pose> point_right;
		point_right.frame_id_ = laser_msg.header.frame_id;
		point_right.stamp_ = laser_msg.header.stamp;
		point_right.setIdentity();
		point_right.setOrigin(tf::Vector3( r_right * cos( t_right ),r_right * sin( t_right ), 0));

		double width = getEulerDistance(point_left,point_right);
		//the distance between the left point and right point should fullfill with the width of the leg;
		if(interval(width, limits.leg_width_lower,limits.leg_width_upper))
		{
			//we found one leg candinate
			leg_candidates.push_back(Leg (point_left,point_right,left,right,false));
			matchingBedlegs(leg_candidates,limits);
			//ROS_INFO("matchingBedlegs finish");
		}
		i = right;
	}

}

void ObjectsDetector::matchingBedlegs(std::vector<Leg> &leg_candidates,SickbedLimits &limits)
{
	//ROS_INFO("leg size %d",int(leg_candidates.size()));
	for(unsigned int i = 0; i< leg_candidates.size()-1;++i)
	{
		//two legs which can make paar fisrtly should be parallel to each other
		if(limits.check_parallel)
		{
			if(leg_candidates[i].center.getRotation().angle(leg_candidates.back().center.getRotation())>0.45)
			{
				//ROS_ERROR("%f,legs is not parallel",leg_candidates[i].center.getRotation().angleShortestPath(leg_candidates.back().center.getRotation()));
				continue;

			}
			double th = std::atan2(leg_candidates[i].center.getOrigin().y()-leg_candidates.back().center.getOrigin().y(),leg_candidates[i].center.getOrigin().x()-leg_candidates.back().center.getOrigin().x());


			if (leg_candidates[i].center.getRotation().angle(tf::createQuaternionFromYaw(th))>0.45)
			{
				continue;
			}
		}
		//ROS_INFO("check leg %d",i);
		double distance = getEulerDistance(leg_candidates[i].center,leg_candidates.back().center);
		//ROS_INFO("legs distance %f",distance);
		bool find_sickbed = false;


		if(interval(distance, limits.leggap_length_lower,limits.leggap_length_upper))
		{

			//ROS_INFO("length fulfill");


			LegPaar legpaar(&leg_candidates[i],&(leg_candidates.back()));
			std::list<LegPaar>::iterator it = legpaar_length_candidates.begin();
			for(;it!=legpaar_length_candidates.end();)
			{
				distance = getEulerDistance((*it).center,legpaar.center);
				//ROS_INFO("paar width distance %f",distance);
				if(interval(distance, limits.leggap_width_lower,limits.leggap_width_upper))
				{
					//ROS_INFO("width fulfill");
					Sickbed sickbed((*it),legpaar,0);
					if(validateSickBed(sickbed,limits))
					{
						sickbedlist.push_back(sickbed);
						//ROS_INFO("find width bed");
						it=legpaar_length_candidates.erase(it);
						find_sickbed = true;

					}
					else
					{
						++it;
					}
				}
				else
				{
					++it;
				}
			}
			if(!find_sickbed)
				legpaar_length_candidates.push_back(legpaar);

		}
		else if(interval(distance, limits.leggap_width_lower,limits.leggap_width_upper))
		{
			//ROS_INFO("width fulfill");
			LegPaar legpaar(&leg_candidates[i],&(leg_candidates.back()));
			std::list<LegPaar>::iterator it = legpaar_width_candidates.begin();
			for(;it!=legpaar_width_candidates.end();)
			{
				distance = getEulerDistance((*it).center,legpaar.center);
				//ROS_INFO("paar length distance %f",distance);
				if(interval(distance, limits.leggap_length_lower,limits.leggap_length_upper))
				{
					//ROS_INFO("length fulfill");
					Sickbed sickbed((*it),legpaar,1);
					if(validateSickBed(sickbed,limits))
					{
						sickbedlist.push_back(sickbed);
						//ROS_INFO("find length bed");
						it=legpaar_width_candidates.erase(it);
						find_sickbed = true;
					}
					else
					{
						++it;
					}
				}
				else
				{
					++it;
				}
			}
			if(!find_sickbed)
				legpaar_width_candidates.push_back(legpaar);
		}
	}

}
bool ObjectsDetector::validateSickBed(Sickbed &sickbed,SickbedLimits &limits)
{
	double dist1 = getEulerDistance(sickbed.paar1.leg1->center,sickbed.paar2.leg1->center);
	double dist2 = getEulerDistance(sickbed.paar1.leg2->center,sickbed.paar2.leg2->center);
	//double dot = (sickbed.paar1.leg1->center.getOrigin() -sickbed.paar1.leg2->center.getOrigin()).dot(sickbed.paar1.leg1->center.getOrigin() -sickbed.paar1.leg2->center.getOrigin());
	//ROS_ERROR("dot 1 %f",dot);
	//if(fabs(dot)>limits.dot_limit)
	//{
	//	return false;
	//}
	//dot = (sickbed.paar2.leg2->center.getOrigin() -sickbed.paar1.leg2->center.getOrigin()).dot(sickbed.paar2.leg2->center.getOrigin() -sickbed.paar2.leg1->center.getOrigin());
	//ROS_ERROR("dot 2 %f",dot);
	//if(fabs(dot)>limits.dot_limit)
	//{
	//	return false;
	//}
	if(sickbed.type == 0)// check length
	{
		return interval(dist1, limits.leggap_length_lower,limits.leggap_length_upper)&&interval(dist2, limits.leggap_length_lower,limits.leggap_length_upper);
	}
	else //check width
	{

		return interval(dist1, limits.leggap_width_lower,limits.leggap_width_upper)&&interval(dist2, limits.leggap_width_lower,limits.leggap_width_upper);
	}
}
bool ObjectsDetector::createJsonString(std::vector<Sickbed> &objectslist,std::string &json_string)
{
	if(objectslist.empty())
	{
		return false;
	}
	Json::Value root ;

	for(std::vector<Sickbed>::iterator it = objectslist.begin(); it != objectslist.end();++it )
	{
		Json::Value json;

		if(createJsonString(*it,json))
		{
			root[OBJECTS].append(json);
		}
	}

	Json::StyledWriter writer;
	json_string = writer.write(root);
	return true;
}
bool ObjectsDetector::createJsonString(Sickbed &object,Json::Value &json)
{
	double x,y,th;
	geometry_msgs::PoseStamped pose_tf;
	std::vector<geometry_msgs::Point> oriented_footprint;
	tf::poseStampedTFToMsg(object.center,pose_tf);
	x = object.center.getOrigin().getX();
	y = object.center.getOrigin().getY();
	th = tf::getYaw(object.center.getRotation());
	costmap_2d::transformFootprint(x,y,th,sickbed_footprint_,oriented_footprint);
	json[FRAME_ID] = "map";
	json[STAMP] = object.center.stamp_.toSec();
	json[TYPE]  = 2;
	json[DURATION] = 1.0;
	try
	{
		tf_->waitForTransform("map",object.center.frame_id_,object.center.stamp_,ros::Duration(0.1));
		std::vector<geometry_msgs::PoseStamped> oriented_footprint_tf;
		oriented_footprint_tf.resize(oriented_footprint.size(),pose_tf);

		for(int i = 0;i<oriented_footprint.size();++i)
		{
			oriented_footprint_tf[i].pose.position = oriented_footprint[i];
			tf_->transformPose("map",oriented_footprint_tf[i],oriented_footprint_tf[i]);

			Json::Value pose;
			pose.append(oriented_footprint_tf[i].pose.position.x);
			pose.append(oriented_footprint_tf[i].pose.position.y);
			pose.append(oriented_footprint_tf[i].pose.position.z);
			json[FOOTPRINT].append(pose);

		}
	}
	catch(tf::TransformException& ex)
	{
		return false;
	}


	return true;
}
//int main(int argc, char **argv)
//{
//	ros::init(argc, argv, "objects_detector");
//	tf::TransformListener tf;
//	ObjectsDetector detector(&tf);
//
//	ros::Rate r(10);
//	while(ros::ok())
//	{
//		//ros::Time start = ros::Time::now();
//		detector.detect();
//		//ROS_INFO("Objects Detector takes %7.6f sceond.", (ros::Time::now()-start).toSec());
//		ros::spinOnce();
//		r.sleep();
//	}
//
//}


