/*
 * radius_filter.h
 *
 *  Created on: Mar 17, 2017
 *      Author: tomzhang
 */

#ifndef INCLUDE_LASER_FILTERS_RADIUS_FILTER_H_
#define INCLUDE_LASER_FILTERS_RADIUS_FILTER_H_


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"
#include "angles/angles.h"

namespace laser_filters
{

class LaserRadiusFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

	double radius_ ;
	int min_neighbors_;
	double lower_threshold_ ;
	double upper_threshold_;
	int shift_points_;

	bool configure()
	{
		radius_ = 0.0;
		min_neighbors_ = 0;
		lower_threshold_ = 0;
		upper_threshold_ = 999999;

		shift_points_ = 0;

		getParam("radius", radius_);
		getParam("min_neighbors", min_neighbors_);
		getParam("lower_threshold", lower_threshold_) ;
		getParam("upper_threshold", upper_threshold_) ;
		getParam("shift_points",shift_points_);
		return true;
	}

	virtual ~LaserRadiusFilter()
	{

	}

	inline double getDistance(const double a,const double b, const double included_angle)
	{
		return std::sqrt(a*a+b*b-2*a*b*std::cos(included_angle));
	}
	bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
	{
		ROS_DEBUG("laser filter radius start");
		filtered_scan = input_scan;
		std::vector<int> number_of_neigbors;// count the points' neighbors
		number_of_neigbors.resize(input_scan.ranges.size(),0);
		ROS_DEBUG("laser filter radius number_of_neigbors size %lu",input_scan.ranges.size());
		for (unsigned int i=0; i < filtered_scan.ranges.size();i++) // Need to check ever reading in the current scan
		{
			//ROS_INFO("i:%d,range:%f,num:%d,an:%f",i,filtered_scan.ranges[i],number_of_neigbors[i],filtered_scan.angle_increment);
			if(filtered_scan.ranges[i]<lower_threshold_)
			{
				number_of_neigbors[i]=-1; //the point will be not handle
				continue;
			}
			if(upper_threshold_<filtered_scan.ranges[i])
			{
				number_of_neigbors[i]=-1; //the point will be not handle
				continue;
			}
			for (unsigned int j=i+1; j < filtered_scan.ranges.size();j++)
			{
				if(filtered_scan.ranges[j]<lower_threshold_)
				{
					continue;
				}
				if(upper_threshold_<filtered_scan.ranges[j])
				{
					continue;
				}
				if(radius_>getDistance(filtered_scan.ranges[i],filtered_scan.ranges[j],(j-i)*filtered_scan.angle_increment))
				{
					number_of_neigbors[i]++;
					number_of_neigbors[j]++;
				}
			}


		}

		bool is_assigned_value = false;
		for (unsigned int i = 0;i<filtered_scan.ranges.size();i++)
		{

			if(number_of_neigbors[i]<min_neighbors_&&number_of_neigbors[i]>-1)
			{
				unsigned int l,r;
				for(unsigned int j = 1; j <= shift_points_; j++)
				{
					l = std::max<int>(i-j,0);
					if(number_of_neigbors[l]>=min_neighbors_)
					{
						filtered_scan.ranges[i] = filtered_scan.ranges[l];
						is_assigned_value = true;
						break;
					}
					r = std::min<int>(i+j,filtered_scan.ranges.size()-1);
					if(number_of_neigbors[r]>=min_neighbors_)
					{
						filtered_scan.ranges[i] = filtered_scan.ranges[r];
						is_assigned_value = true;
						break;
					}
				}

				/*
				for(unsigned int l =std::max<int>(i-1, 0),r = std::min<int>(i+1, filtered_scan.ranges.size()-1); l>=std::max<int>(i-shift_points_, 0),r<=std::min<int>(i+shift_points_, filtered_scan.ranges.size()-1);)
				{
					if(number_of_neigbors[r]>=min_neighbors_)
					{
						filtered_scan.ranges[i] = filtered_scan.ranges[r];
						is_assigned_value = true;
						break;
					}
					if(number_of_neigbors[l]>=min_neighbors_)
					{
						filtered_scan.ranges[i] = filtered_scan.ranges[l];
						is_assigned_value = true;
						break;
					}
					if(l>0)
					{
						l--;
					}
					if(r<filtered_scan.ranges.size()-1)
						r++;
				}
				*/
				if(false == is_assigned_value)
				{
					filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();  // Failed test to set the ranges to invalid value
				}
				else
				{
					is_assigned_value = false;
				}
				//ROS_INFO("%d.range:%f",i,filtered_scan.ranges[i]);
			}
			else
			{
				continue;
			}

		}
		ROS_DEBUG("laser filter radius end filtered_scan size %lu",filtered_scan.ranges.size());
		return true;
	}
} ;

}



#endif /* INCLUDE_LASER_FILTERS_RADIUS_FILTER_H_ */
