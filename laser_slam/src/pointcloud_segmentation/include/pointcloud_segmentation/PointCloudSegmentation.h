/*
 * PointCloudSegmentation.h
 *
 *  Created on: 2017年1月3日
 *      Author: tomzhang
 */
#ifndef INCLUDE_POINTCLOUD_SEGMENTATION_POINTCLOUDSEGMENTATION_H_
#define INCLUDE_POINTCLOUD_SEGMENTATION_POINTCLOUDSEGMENTATION_H_

#include <stdio.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pointcloud_segmentation/PointOrganizator.h>
#include <visualization_msgs/Marker.h>
namespace pointcloud_segmentation{
class PointCloudSegmentation
{
public:
	PointCloudSegmentation(
			const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int rows,
			const int cols, const float camera_x, const float camera_y,
			const float camera_z, const float camera_view,const float camera_pitch_,
			const float obstacle_height_threshold,
			const float dist_threshold, const float slope_threshold) ;
	~PointCloudSegmentation();
	void segmentation( pcl::PointCloud<pcl::PointXYZ>&obstacle_cloud,pcl::PointCloud<pcl::PointXYZ> &ground_cloud);

	void visualization(visualization_msgs::Marker &points_g,visualization_msgs::Marker &points_o);
private:
	void point_segment(pcl::PointCloud<pcl::PointXYZ>&obstacle_cloud,pcl::PointCloud<pcl::PointXYZ> &ground_cloud);
	void obstacle_height_test( float &height, int &start_idx, int end_idx, const int col_idx);
	void calculate_angle();
	void calculate_delta_r(const float r, const float h, const float delta_pitch, float &delta_r);
	void calculate_delta_r_threshold(const float r, const float h, const float angle_threshold, const float delta_pitch, float &delta_r);
	void calculate_threshold(const float r, const float delta_pitch, float &delta_r);
	void calculate_unevenness();
	std::vector<Point3D> ground,obstacle;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	const PointOrganizator organizator_;
	const int rows_, cols_;
	const Point3D camera_point_;
	const float camera_view_,camera_pitch_,obstacle_height_threshold_,dist_threshold_,slope_threshold_;
};
}
#endif /* INCLUDE_POINTCLOUD_SEGMENTATION_POINTCLOUDSEGMENTATION_H_ */
