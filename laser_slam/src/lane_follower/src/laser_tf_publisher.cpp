/*
 * laser_publisher.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tomzhang
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#define LASER_NUM 13
static std::string laser_frames[LASER_NUM] = {"laser_frame_0","laser_frame_1","laser_frame_2","laser_frame_3","laser_frame_4","laser_frame_5",
										      "laser_frame_6","laser_frame_7","laser_frame_8","laser_frame_9","laser_frame_10","laser_frame_11","laser_frame_12"};

static double y[LASER_NUM] = {-0.24,-0.24,-0.24,0.24,0.24,0.24,-0.21,-0.08,0.08,0.21,-0.15,0.0,0.15};
static double x[LASER_NUM] = {0.29,-0.15,-0.45,0.29,-0.15,-0.45,-0.5,-0.5,-0.5,-0.5,0.2,0.2,0.2};
//static double x[LASER_NUM-3] = {-0.24,-0.24,-0.24,0.24,0.24,0.24,-0.21,-0.08,0.08,0.21};
//static double y[LASER_NUM-3] = {0.29,-0.15,-0.45,0.29,-0.15,-0.45,-0.5,-0.5,-0.5,-0.5};
//static double th[LASER_NUM-3] = {M_PI/2,M_PI/2,M_PI/2,-M_PI/2,-M_PI/2,-M_PI/2,M_PI,M_PI,M_PI,M_PI};
static double th[LASER_NUM] = {-M_PI/2,-M_PI/2,-M_PI/2,M_PI/2,M_PI/2,M_PI/2,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI};

//static double th[LASER_NUM-3] = {M_PI,M_PI,M_PI,0.0,0.0,0.0,M_PI/2,M_PI/2,M_PI/2,M_PI/2};
namespace lane_follower
{


class LaserPublisher
{
public:

	LaserPublisher()
	{

		ros::NodeHandle n;
		//pub_ = n.advertise<sensor_msgs::PointCloud2>("lasercloud",50);
		//lasers_.resize(LASER_NUM);
		//initCloud();
	}
	~LaserPublisher()
	{

	}
	//void createLaser(int laser_num)
	//{
	//	for(int i = 0; i< laser_num; i++)
	//	{
	//		//lasers_[i]=(((double)i+10.0)/10.0);
	//		lasers_[i]=1.2;
	//	}
	//}
	void createTransform(int frame_num)
	{
		for(int i = 0; i<frame_num; i++)
		{
		     tf::Transform transform;
		     tf::Quaternion q;
		     q.setRPY(0, 0, th[i]);
		     transform.setRotation(q);
		     transform.setOrigin( tf::Vector3(x[i], y[i], 0.0) );

		     trans_vec.push_back(transform);
		}


	}
	void publishCloud()
	{
		//cloud_out.header.stamp = ros::Time::now();
		for(int i=0;i<LASER_NUM-3;i++)
		{
		    	br.sendTransform(tf::StampedTransform(trans_vec[i], ros::Time::now(), "base_link", laser_frames[i]));
		    	//cloud_out.header.frame_id = laser_frames[i];
		    	//float *pstep = (float*)&cloud_out.data[0];
		    	//pstep[0] = (float)lasers_[i];
		    	//pstep[1] = 0.0;
		    	//pub_.publish(cloud_out);
		}

	}
private:
	void initCloud()
	{
		cloud_out.height = 1;
		cloud_out.width  = 1;
		cloud_out.fields.resize (4);
		cloud_out.fields[0].name = "x";
		cloud_out.fields[0].offset = 0;
		cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
		cloud_out.fields[0].count = cloud_out.width;
		cloud_out.fields[1].name = "y";
		cloud_out.fields[1].offset = 4;
		cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
		cloud_out.fields[1].count = cloud_out.width;
		cloud_out.fields[2].name = "z";
		cloud_out.fields[2].offset = 8;
		cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
		cloud_out.fields[2].count = cloud_out.width;
		cloud_out.fields[3].name = "intensity";
		cloud_out.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
		cloud_out.fields[3].offset = 12;
		cloud_out.fields[3].count = cloud_out.width;
	    cloud_out.point_step = 16;
		cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
		cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
		cloud_out.is_dense = false;
	}
	ros::Publisher pub_;
	std::vector<double> lasers_;
	std::vector<tf::Transform> trans_vec;
	tf::TransformBroadcaster br;
        sensor_msgs::PointCloud2 cloud_out;
};
};


int main(int argc, char** argv){
  ros::init(argc, argv,"laser_tf_pub");
  lane_follower::LaserPublisher laser_tf_pub;
  laser_tf_pub.createTransform(LASER_NUM);
  ros::Rate r(10);
  while(ros::ok())
  {
	  //laserpub.createLaser(LASER_NUM-3);
	  laser_tf_pub.publishCloud();
	  r.sleep();
  }


  return(0);
}


