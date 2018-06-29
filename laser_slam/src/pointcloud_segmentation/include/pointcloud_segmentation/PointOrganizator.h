/*
 * PointOrganizator.h
 *
 *  Created on: 2017年1月3日
 *      Author: tomzhang
 */

#ifndef INCLUDE_POINTCLOUD_SEGMENTATION_POINTORGANIZATOR_H_
#define INCLUDE_POINTCLOUD_SEGMENTATION_POINTORGANIZATOR_H_


#include <cstdio>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pointcloud_segmentation/Point3D.h>
namespace pointcloud_segmentation{
class PointOrganizator
{
public:
	PointOrganizator(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int rows, int cols);
	~PointOrganizator();
	int &rows();
	int &cols();
    std::vector<std::vector<Point3D> > points;
private:
	void Organizator();
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    int rows_, cols_;
};

}

#endif /* INCLUDE_POINTCLOUD_SEGMENTATION_POINTORGANIZATOR_H_ */
