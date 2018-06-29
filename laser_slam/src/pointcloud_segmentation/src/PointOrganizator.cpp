/*
 * PointOrganizator.cpp
 *
 *  Created on: 2017年1月3日
 *      Author: tomzhang
 */
#include "ros/ros.h"
#include "pointcloud_segmentation/PointOrganizator.h"
using namespace pointcloud_segmentation;
PointOrganizator::PointOrganizator(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int rows, int cols) :
		cloud_(cloud), rows_(rows), cols_(cols)
{
	Organizator();
	//std::cout<<<<std::endl;
	//ROS_INFO("points:%d",(int)(points.size()));
}
PointOrganizator::~PointOrganizator(){

}
void PointOrganizator::Organizator()
{

	/*
	for (int i = 0; i < rows_; i++)
	{
		std::vector<Point3D> points_in_one_row;
		points_in_one_row.reserve(cols_);
		for (int j = 0; j < cols_; j++)
		{
			//ROS_INFO("%d,%d  x:%f, y:%f, z:%f.",i,j,cloud_->points[cols_ * i + j].x,cloud_->points[cols_ * i + j].y,cloud_->points[cols_ * i + j].z);

			Point3D temp(cloud_->points[cols_ * i + j].x,
					cloud_->points[cols_ * i + j].y,
					cloud_->points[cols_ * i + j].z);
			temp.index_ =cols_ * i + j;
			points_in_one_row.push_back(temp);
		}

		//ROS_INFO("points_in_one_row.size:%d",(int)(points_in_one_row.size()));

		points.push_back(points_in_one_row);
	}	*/
	for(int j =0;j<cols_;j++){
		std::vector<Point3D> points_in_one_col;
		points_in_one_col.reserve(rows_);
		for(int i = 0; i<rows_;i ++){
			Point3D temp(cloud_->points[cols_ * i + j].x,
								cloud_->points[cols_ * i + j].y,
								cloud_->points[cols_ * i + j].z);
						temp.index_ =cols_ * i + j;
						points_in_one_col.push_back(temp);
		}
		points.push_back(points_in_one_col);
	}

}

