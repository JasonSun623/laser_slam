/*
 * velocity_damper.cpp
 *
 *  Created on: Mar 21, 2017
 *      Author: tomzhang
 */
#include "velocity_damper/velocity_damper.h"
#define COL 13
#define ROW 3
#define SAMPLE_NUM 5
//static double damper_factors[ROW][COL] = {
//		{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,},
//		{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,},
//		{1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00}
//};

static double damper_factors[ROW][COL] = {  //mid
		{0.00, 0.00, 0.00, 0.01, 0.02, 0.02, 0.03, 0.02, 0.02, 0.01, 0.00, 0.00, 0.00,},
		{0.00, 0.00, 0.00, 0.01, 0.01, 0.02, 0.02, 0.02, 0.01, 0.01, 0.00, 0.00, 0.00,},
		{0.00, 0.00, 0.00, 0.00, 0.01, 0.01, 0.02, 0.01, 0.01, 0.00, 0.00, 0.00, 0.00}
};
static int damper_matrix[ROW*COL];
namespace velocity_damper{

VelocityDamper::VelocityDamper(std::string name, tf::TransformListener *tf):name_(name), tf_(tf),filter_init_(false)
{
	onInitialize();
}
VelocityDamper::~VelocityDamper()
{

}
void VelocityDamper::onInitialize()
{
	ros::NodeHandle nh("~/" + name_), g_nh;

	std::string topic, sensor_frame, data_type;
	topic = "scan_filtered";
	sensor_frame = "base_link";
	data_type = "LaserScan";

	if (data_type == "LaserScan")
	{
		boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
		> sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

		boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
		> filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, sensor_frame, 50));


		filter->registerCallback(
				boost::bind(&VelocityDamper::laserScanCallback, this, _1));


		observation_subscribers_.push_back(sub);
		observation_notifiers_.push_back(filter);

		observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
	}
	topic = "vel";
	sub_ = g_nh.subscribe("vel",10,&VelocityDamper::velocityCallback,this);



	observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
	pub_ = g_nh.advertise<geometry_msgs::Twist>("nav_pri_cmdvel", 1);

}

void VelocityDamper::velocityCallback(const geometry_msgs::TwistConstPtr & message)
{
	if(filter_init_ ==false) return;
	geometry_msgs::Twist vel;
	mtx_.lock();
	vel.linear.x = message->linear.x*factor_;
	vel.angular.z = message->angular.z*factor_;
	mtx_.unlock();
	pub_.publish(vel);

}
void VelocityDamper::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message)
{
	double angle = message->angle_increment*message->ranges.size();
	double step_angle = angle/COL;
	int col = 0;
	int row = 0;
	double current_angle_area = 0.0;
	double factor = 0.0;
	memset(damper_matrix,0,sizeof(int)*ROW*COL);
	for(int i =0; i< message->ranges.size() ;i++)
	{

		//ROS_INFO("inc:%f,cur:%f",i*message->angle_increment,current_angle_area);
		if(i*message->angle_increment>current_angle_area+step_angle)
		{
			current_angle_area+=step_angle;
			col++;
		}
		if(message->ranges[i]>3.0)
		{
			continue;
		}
		else if(message->ranges[i]>2.0&&message->ranges[i]<=3.0)
		{
			row = 0;
		}
		else if(message->ranges[i]>1.0&&message->ranges[i]<=2.0)
		{
			row = 1;
		}
		else if(message->ranges[i]>0.0&&message->ranges[i]<=1.0)
		{
			row = 2;
		}
		damper_matrix[row*COL+col]++;
	}


	for(int j =0; j<COL; j++)
	{
		for(int i = ROW-1; i>=0;i--)
		{
			if(damper_matrix[i*COL+j]>=3)
			{
				for(;i>=0;i--)
				{
					factor+=damper_factors[i][j];
				}
			}
		}

	}

	if(false==filter_init_)
	{
		if(SAMPLE_NUM==avg_mean_filter_.size())
		{
			filter_init_=true;
			factor_ /= SAMPLE_NUM;
		}
		else
		{
			avg_mean_filter_.push_back(1-factor);
			factor_+=(1-factor);
		}
	}
	else
	{
		mtx_.lock();
		factor_+=(1-factor-avg_mean_filter_.front())/SAMPLE_NUM;
		mtx_.unlock();
		avg_mean_filter_.erase(avg_mean_filter_.begin());
		avg_mean_filter_.push_back(1-factor);
	}
	ROS_DEBUG("factor:%f",factor_);
	ros::param::set("factor",factor_);


}
void VelocityDamper::setupDynamicReconfigure(ros::NodeHandle& nh)
{

}
void VelocityDamper::reconfigureCB()
{

}

}//namespace


