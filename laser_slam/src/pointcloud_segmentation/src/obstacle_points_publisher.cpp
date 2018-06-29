#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include<time.h>
//  pcl specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>  
#include <pcl/point_types.h> 
#include <pcl/exceptions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/passthrough.h>
#include "pointcloud_segmentation/PointCloudSegmentation.h"
#include <dynamic_reconfigure/server.h>
#include "pointcloud_segmentation/pcsConfig.h"
using namespace pointcloud_segmentation;
class ObstaclePointsPublish
{
public:
	ObstaclePointsPublish(tf::TransformListener &tf):tf_(tf)
{

		pub_obstacle_ = n_.advertise<sensor_msgs::PointCloud2>("obstacle_points", 5);
		pub_ground_ = n_.advertise<sensor_msgs::PointCloud2> ("ground_points",5);
		pc_visualization_ = n_.advertise<visualization_msgs::Marker>(
				"obstacle_points_publish/points_visualization", 5);
		//pub_obstacle_ = n_.advertise<visualization_msgs::Marker>("visualization_obstalce", 1);

		f = boost::bind(&ObstaclePointsPublish::configCallback, this, _1, _2);
		srv.setCallback(f);
		sub_ = n_.subscribe("/camera/depth/points", 5,
				&ObstaclePointsPublish::SegmentationCallback, this);

}
	~ObstaclePointsPublish()
	{
	}
	void configCallback(pointcloud_segmentation::pcsConfig &config, uint32_t level)
	{

		camera_cols_param_ = config.camera_cols_param;
		camera_rows_param_ = config.camera_rows_param;
		camera_point_x_param_ = config.camera_point_x_param;
		camera_point_y_param_ = config.camera_point_y_param;
		camera_point_z_param_ = config.camera_point_z_param;
		camera_view_angle_param_ = config.camera_view_angle_param;
		camera_pitch_param_ = config.camera_pitch_param;
		camera_roll_param_ = config.camera_roll_param;
		camera_sampling_rate_param_ = config.camera_sampling_rate_param;
		slope_threshold_param_ = config.slope_threshold_param;
		obstacle_height_threshold_param_ =
				config.obstacle_height_threshold_param;
		dist_threshold_param_ = config.dist_threshold_param;
		max_x_param_ = config.max_x_param;
	}
	void SegmentationCallback(sensor_msgs::PointCloud2 input_cloud_msg)
	{
		sensor_msgs::PointCloud2 transformed_cloud;
		ros::Time now = ros::Time::now();
		tf_.waitForTransform( "base_link","camera_depth_optical_frame", now, ros::Duration(0.5));
		tf::StampedTransform transform;
		tf_.lookupTransform("base_link","camera_depth_optical_frame",ros::Time(0),transform);
		pcl_ros::transformPointCloud("base_link",transform, input_cloud_msg, transformed_cloud);

		int height = transformed_cloud.height;
		int width = transformed_cloud.width;
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		try
		{
			pcl::PCLPointCloud2 pcl_pc2;
			pcl_conversions::toPCL(transformed_cloud, pcl_pc2);
			pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
		} catch (pcl::PCLException & ex)
		{
			ROS_ERROR("Failed to convert a message to PCL");
			return;
		}

		// filter point cloud for smoothing planar
		pcl::PointCloud<pcl::PointXYZ> xyz_filtered;

		pcl::FastBilateralFilter<pcl::PointXYZ> fbf;
		fbf.setInputCloud(pcl_cloud.makeShared());
		fbf.setSigmaR(0.05);
		fbf.setSigmaS(5.0);

		fbf.filter (xyz_filtered);

		//float camera_h = camera_point_z_param_;
		//float camera_pitch = camera_pitch_param_ / 180.0 * M_PI;

		// sample point cloud for speed up
		unsigned int count = 0;
		pcl::PointCloud<pcl::PointXYZ> sampled_cloud;
		sampled_cloud.points.resize(
				height / camera_sampling_rate_param_* width
				/ camera_sampling_rate_param_);
		sampled_cloud.header.frame_id=pcl_cloud.header.frame_id;

		sampled_cloud.header.stamp=pcl_cloud.header.stamp;
		for (int i = 0; i < height; i++)
		{
			if (i % (camera_sampling_rate_param_) != 0)
				continue;
			for (int j = 0; j < width; j++)
			{
				if (j % camera_sampling_rate_param_ == 0)
				{
					sampled_cloud.points[count] =
							pcl_cloud.points[i * width + j];
					count++;
				}
			}
		}

		// segment point cloud to 2 point cloud: ground and obstacle

		PointCloudSegmentation ps(sampled_cloud.makeShared(),
				height / camera_sampling_rate_param_,
				width / camera_sampling_rate_param_, camera_point_x_param_,
				camera_point_y_param_, camera_point_z_param_,
				camera_view_angle_param_,camera_pitch_param_, obstacle_height_threshold_param_,
				dist_threshold_param_, slope_threshold_param_);

		pcl::PointCloud<pcl::PointXYZ> obstacle_cloud, ground_cloud;
		obstacle_cloud.header.frame_id = transformed_cloud.header.frame_id;
		ground_cloud.header.frame_id = transformed_cloud.header.frame_id;
		obstacle_cloud.header.stamp = pcl_cloud.header.stamp;
		ground_cloud.header.stamp = pcl_cloud.header.stamp;
		ps.segmentation(obstacle_cloud, ground_cloud);

		//pass filter
		pcl::PassThrough<pcl::PointXYZ>pass;
		pass.setInputCloud(obstacle_cloud.makeShared());
		pass.setFilterFieldName("x");
		pass.setFilterLimits(0.0,max_x_param_);
		pass.filter(xyz_filtered);

		//radius filter remove outliers
		pcl::PointCloud<pcl::PointXYZ> radius_filtered_obs;
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
		ror.setInputCloud(xyz_filtered.makeShared());
		ror.setRadiusSearch(0.8);
		ror.setMinNeighborsInRadius(2);
		ror.filter (radius_filtered_obs);

		//convert pcl to msg format
		sensor_msgs::PointCloud2 output_cloud_obs;
		sensor_msgs::PointCloud2 output_cloud_grd;
		try
		{
			pcl::PCLPointCloud2 pcl_pc2_obs;
			toPCLPointCloud2(radius_filtered_obs,pcl_pc2_obs);
			pcl_conversions::fromPCL(pcl_pc2_obs, output_cloud_obs);
			pcl::PCLPointCloud2 pcl_pc2_grd;
			toPCLPointCloud2(ground_cloud,pcl_pc2_grd);
			pcl_conversions::fromPCL(pcl_pc2_grd, output_cloud_grd);
		} catch (pcl::PCLException & ex)
		{
			ROS_ERROR("Failed to convert a PCL to message");
			return;
		}

		pub_obstacle_.publish(output_cloud_obs);
		pub_ground_.publish(output_cloud_grd);

		// visualization for debug
		if(false){
			visualization_msgs::Marker points_g, points_o;
			points_g.header.frame_id = points_o.header.frame_id =
					transformed_cloud.header.frame_id;
			points_g.header.stamp = points_o.header.stamp =
					transformed_cloud.header.stamp;
			ps.visualization(points_g, points_o);


			pc_visualization_.publish(points_g);
			pc_visualization_.publish(points_o);
		}



	}

private:
	ros::NodeHandle n_;

	dynamic_reconfigure::Server<pointcloud_segmentation::pcsConfig> srv;
	dynamic_reconfigure::Server<pointcloud_segmentation::pcsConfig>::CallbackType f;
	tf::TransformListener& tf_;
	ros::Publisher pub_obstacle_;
	ros::Publisher pub_ground_;
	ros::Publisher pc_visualization_;
	ros::Subscriber sub_;

	std::string sensor_frame_;
	std::string robot_base_frame_;

	int camera_cols_param_;
	int camera_rows_param_;
	float camera_point_x_param_;
	float camera_point_y_param_;
	float camera_point_z_param_;
	float camera_view_angle_param_;
	float camera_pitch_param_;
	float camera_roll_param_;
	int camera_sampling_rate_param_;
	float slope_threshold_param_;
	float obstacle_height_threshold_param_;
	float dist_threshold_param_;
	float max_x_param_;
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "obstacle_points_publish");
	tf::TransformListener tf(ros::Duration(10));

	ObstaclePointsPublish oppObj(tf);

	ros::spin();

	return 0;
}
