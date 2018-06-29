/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/rgbd_voxel_layer.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

#define VOXEL_BITS 16
PLUGINLIB_EXPORT_CLASS(costmap_2d::RgbdVoxelLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void RgbdVoxelLayer::onInitialize()
{
	//ObstacleLayer::onInitialize();

	ros::NodeHandle nh("~/" + name_), g_nh;

	rolling_window_ = layered_costmap_->isRolling();
	bool track_unknown_space;
	nh.param("voxel_track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
	if (track_unknown_space)
		default_value_ = NO_INFORMATION;
	else
		default_value_ = FREE_SPACE;
	//ROS_INFO("obstacle layer default value:%d",default_value_);
	ObstacleLayer::matchSize();
	current_ = true;

	global_frame_ = layered_costmap_->getGlobalFrameID();
	double transform_tolerance;
	nh.param("transform_tolerance", transform_tolerance, 0.2);

	std::string topics_string;
	// get the topics that we'll subscribe to from the parameter server
	nh.param("voxel_observation_sources", topics_string, std::string(""));
	nh.param("keep_history", keep_history_, false);
	// get our tf prefix
	ros::NodeHandle prefix_nh;
	const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

	// now we need to split the topics based on whitespace which we can use a stringstream for
	std::stringstream ss(topics_string);

	std::string source;

	while (ss >> source)
	{
		ros::NodeHandle source_node(nh, source);

		// get the parameters for the specific topic
		double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
		std::string topic, sensor_frame, data_type, ground_topic;
		bool inf_is_valid, clearing, marking, use_ground_clearing;

		source_node.param("topic", topic, source);
		source_node.param("sensor_frame", sensor_frame, std::string(""));
		source_node.param("observation_persistence", observation_keep_time, 0.0);
		source_node.param("expected_update_rate", expected_update_rate, 0.0);
		source_node.param("data_type", data_type, std::string("PointCloud2"));
		source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
		source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
		source_node.param("inf_is_valid", inf_is_valid, false);
		source_node.param("clearing", clearing, true);
		source_node.param("marking", marking, true);
		source_node.param("use_ground_clearing", use_ground_clearing, false);
		source_node.param("ground_topic", ground_topic, std::string("ground_points"));
		ROS_INFO("min max obstacle height:%f,%f",min_obstacle_height,max_obstacle_height);

		if (!sensor_frame.empty())
		{
			sensor_frame = tf::resolve(tf_prefix, sensor_frame);
		}

		if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
		{
			ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
			throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
		}

		std::string raytrace_range_param_name, obstacle_range_param_name;

		// get the obstacle range for the sensor
		double obstacle_range = 2.5;
		if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
		{
			source_node.getParam(obstacle_range_param_name, obstacle_range);
		}

		// get the raytrace range for the sensor
		double raytrace_range = 3.0;
		if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
		{
			source_node.getParam(raytrace_range_param_name, raytrace_range);
		}

		ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
				sensor_frame.c_str());

		// create an observation buffer
		observation_buffers_.push_back(
				boost::shared_ptr < ObservationBuffer
				> (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
						max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
						sensor_frame, transform_tolerance)));

		// check if we'll add this buffer to our marking observation buffers
		if (marking)
			marking_buffers_.push_back(observation_buffers_.back());

		// check if we'll also add this buffer to our clearing observation buffers
		if (clearing)
		{
			clearing_buffers_.push_back(observation_buffers_.back());
			if(use_ground_clearing)
			{
				clearing_buffers_.push_back(boost::shared_ptr < ObservationBuffer
						> (new ObservationBuffer(ground_topic, observation_keep_time, expected_update_rate, -0.15,
								0.15, obstacle_range, raytrace_range, *tf_, global_frame_,
								sensor_frame, transform_tolerance)));
			}
		}

		ROS_DEBUG(
				"Created an observation buffer for source %s, topic %s, global frame: %s, "
				"expected update rate: %.2f, observation persistence: %.2f",
				source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

		// create a callback for the topic
		if (data_type == "LaserScan")
		{
			boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
			> sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

			boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
			> filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));

			if (inf_is_valid)
			{
				//gavin: core code to handle laser scan data for costmap obstacle layer
				filter->registerCallback(
						boost::bind(&ObstacleLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
			}
			else
			{
				filter->registerCallback(
						boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));
			}

			observation_subscribers_.push_back(sub);
			observation_notifiers_.push_back(filter);

			observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
		}
		else if (data_type == "PointCloud")
		{
			boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
			> sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

			if (inf_is_valid)
			{
				ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
			}

			boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud>
			> filter(new tf::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50));
			filter->registerCallback(
					boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

			observation_subscribers_.push_back(sub);
			observation_notifiers_.push_back(filter);
		}
		else
		{
			boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
			> sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

			if (inf_is_valid)
			{
				ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
			}

			boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
			> filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50));
			filter->registerCallback(
					boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

			observation_subscribers_.push_back(sub);
			observation_notifiers_.push_back(filter);

			if(use_ground_clearing)
			{
				boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
				> sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

				if (inf_is_valid)
				{
					ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
				}

				boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
				> filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50));
				filter->registerCallback(
						boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, clearing_buffers_.back()));

				observation_subscribers_.push_back(sub);
				observation_notifiers_.push_back(filter);
			}
		}

		if (sensor_frame != "")
		{
			std::vector < std::string > target_frames;
			target_frames.push_back(global_frame_);
			target_frames.push_back(sensor_frame);
			observation_notifiers_.back()->setTargetFrames(target_frames);
		}
	}

	dsrv_ = NULL;
	//ROS_INFO("before setup dynamic reconfigure");
	setupDynamicReconfigure(nh);
	// ROS_INFO("after setup dynamic reconfigure");


	ros::NodeHandle private_nh("~/" + name_);

	private_nh.param("publish_voxel_map", publish_voxel_, false);
	if (publish_voxel_)
		voxel_pub_ = private_nh.advertise < costmap_2d::VoxelGrid > ("voxel_grid", 1);

	clearing_endpoints_pub_ = private_nh.advertise<sensor_msgs::PointCloud>("clearing_endpoints", 1);
}

void RgbdVoxelLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
	voxel_dsrv_ = new dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig>::CallbackType cb = boost::bind(
			&RgbdVoxelLayer::reconfigureCB, this, _1, _2);
	voxel_dsrv_->setCallback(cb);
}

RgbdVoxelLayer::~RgbdVoxelLayer()
{
	if (voxel_dsrv_)
		delete voxel_dsrv_;
}

void RgbdVoxelLayer::reconfigureCB(costmap_2d::VoxelPluginConfig &config, uint32_t level)
{
	enabled_ = config.enabled;
	footprint_clearing_enabled_ = config.footprint_clearing_enabled;
	max_obstacle_height_ = config.max_obstacle_height;
	size_z_ = config.z_voxels;
	origin_z_ = config.origin_z;
	z_resolution_ = config.z_resolution;
	unknown_threshold_ = config.unknown_threshold + (VOXEL_BITS - size_z_);
	mark_threshold_ = config.mark_threshold;
	combination_method_ = config.combination_method;

	matchSize();
}

void RgbdVoxelLayer::matchSize()
{
	ObstacleLayer::matchSize();
	voxel_grid_.resize(size_x_, size_y_, size_z_);
	ROS_ASSERT(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

void RgbdVoxelLayer::reset()
{
	deactivate();
	resetMaps();
	//voxel_grid_.reset();
	activate();
}

void RgbdVoxelLayer::resetMaps()
{
	Costmap2D::resetMaps();
	voxel_grid_.reset();
}

void RgbdVoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
		double* min_y, double* max_x, double* max_y)
{
	if (rolling_window_)
		updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
	if (!enabled_)
		return;
	useExtraBounds(min_x, min_y, max_x, max_y);

	bool current = true;
	std::vector<Observation> observations, clearing_observations;

	// get the marking observations
	current = getMarkingObservations(observations) && current;

	// get the clearing observations
	current = getClearingObservations(clearing_observations) && current;

	// update the global current status
	current_ = current;

	// raytrace freespace
	for (unsigned int i = 0; i < clearing_observations.size(); ++i)
	{
		raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
	}

	// place the new obstacles into a priority queue... each with a priority of zero to begin with
	for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
	{
		const Observation& obs = *it;

		const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

		double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

		for (unsigned int i = 0; i < cloud.points.size(); ++i)
		{
			// if the obstacle is too high or too far away from the robot we won't add it
			if (cloud.points[i].z > max_obstacle_height_)
				continue;

			// compute the squared distance from the hitpoint to the pointcloud's origin
			double sq_dist = (cloud.points[i].x - obs.origin_.x) * (cloud.points[i].x - obs.origin_.x)
        				  + (cloud.points[i].y - obs.origin_.y) * (cloud.points[i].y - obs.origin_.y)
						  + (cloud.points[i].z - obs.origin_.z) * (cloud.points[i].z - obs.origin_.z);

			// if the point is far enough away... we won't consider it
			if (sq_dist >= sq_obstacle_range)
				continue;

			// now we need to compute the map coordinates for the observation
			unsigned int mx, my, mz;
			if (cloud.points[i].z < origin_z_)
			{
				if (!worldToMap3D(cloud.points[i].x, cloud.points[i].y, origin_z_, mx, my, mz))
					continue;
			}
			else if (!worldToMap3D(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, mx, my, mz))
			{
				continue;
			}

			// mark the cell in the voxel grid and check if we should also mark it in the costmap
			if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_))
			{
				unsigned int index = getIndex(mx, my);

				costmap_[index] = LETHAL_OBSTACLE;
				touch((double)cloud.points[i].x, (double)cloud.points[i].y, min_x, min_y, max_x, max_y);
			}
		}
	}

	if (publish_voxel_)
	{
		costmap_2d::VoxelGrid grid_msg;
		unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
		grid_msg.size_x = voxel_grid_.sizeX();
		grid_msg.size_y = voxel_grid_.sizeY();
		grid_msg.size_z = voxel_grid_.sizeZ();
		grid_msg.data.resize(size);
		memcpy(&grid_msg.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

		grid_msg.origin.x = origin_x_;
		grid_msg.origin.y = origin_y_;
		grid_msg.origin.z = origin_z_;

		grid_msg.resolutions.x = resolution_;
		grid_msg.resolutions.y = resolution_;
		grid_msg.resolutions.z = z_resolution_;
		grid_msg.header.frame_id = global_frame_;
		grid_msg.header.stamp = ros::Time::now();
		voxel_pub_.publish(grid_msg);
	}

	updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void RgbdVoxelLayer::clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info)
{
	// get the cell coordinates of the center point of the window
	unsigned int mx, my;
	if (!worldToMap(wx, wy, mx, my))
		return;

	// compute the bounds of the window
	double start_x = wx - w_size_x / 2;
	double start_y = wy - w_size_y / 2;
	double end_x = start_x + w_size_x;
	double end_y = start_y + w_size_y;

	// scale the window based on the bounds of the costmap
	start_x = std::max(origin_x_, start_x);
	start_y = std::max(origin_y_, start_y);

	end_x = std::min(origin_x_ + getSizeInMetersX(), end_x);
	end_y = std::min(origin_y_ + getSizeInMetersY(), end_y);

	// get the map coordinates of the bounds of the window
	unsigned int map_sx, map_sy, map_ex, map_ey;

	// check for legality just in case
	if (!worldToMap(start_x, start_y, map_sx, map_sy) || !worldToMap(end_x, end_y, map_ex, map_ey))
		return;

	// we know that we want to clear all non-lethal obstacles in this window to get it ready for inflation
	unsigned int index = getIndex(map_sx, map_sy);
	unsigned char* current = &costmap_[index];
	for (unsigned int j = map_sy; j <= map_ey; ++j)
	{
		for (unsigned int i = map_sx; i <= map_ex; ++i)
		{
			// if the cell is a lethal obstacle... we'll keep it and queue it, otherwise... we'll clear it
			if (*current != LETHAL_OBSTACLE)
			{
				if (clear_no_info || *current != NO_INFORMATION)
				{
					*current = FREE_SPACE;
					voxel_grid_.clearVoxelColumn(index);
				}
			}
			current++;
			index++;
		}
		current += size_x_ - (map_ex - map_sx) - 1;
		index += size_x_ - (map_ex - map_sx) - 1;
	}
}

void RgbdVoxelLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
		double* max_x, double* max_y)
{
	if (clearing_observation.cloud_->points.size() == 0)
		return;

	double sensor_x, sensor_y, sensor_z;
	double ox = clearing_observation.origin_.x;
	double oy = clearing_observation.origin_.y;
	double oz = clearing_observation.origin_.z;

	if (!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z))
	{
		ROS_WARN_THROTTLE(
				1.0,
				"The origin for the sensor at (%.2f, %.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.%lu",
				ox, oy, oz,clearing_observation.cloud_->header.stamp);
		return;
	}

	bool publish_clearing_points = (clearing_endpoints_pub_.getNumSubscribers() > 0);
	if (publish_clearing_points)
	{
		clearing_endpoints_.points.clear();
		clearing_endpoints_.points.reserve(clearing_observation.cloud_->points.size());
	}

	// we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
	double map_end_x = origin_x_ + getSizeInMetersX();
	double map_end_y = origin_y_ + getSizeInMetersY();

	for (unsigned int i = 0; i < clearing_observation.cloud_->points.size(); ++i)
	{
		double wpx = clearing_observation.cloud_->points[i].x;
		double wpy = clearing_observation.cloud_->points[i].y;
		double wpz = clearing_observation.cloud_->points[i].z;

		double distance = dist(ox, oy, oz, wpx, wpy, wpz);
		double scaling_fact = 1.0;
		scaling_fact = std::max(std::min(scaling_fact, (distance - 2 * resolution_) / distance), 0.0);
		wpx = scaling_fact * (wpx - ox) + ox;
		wpy = scaling_fact * (wpy - oy) + oy;
		wpz = scaling_fact * (wpz - oz) + oz;

		double a = wpx - ox;
		double b = wpy - oy;
		double c = wpz - oz;
		double t = 1.0;

		// we can only raytrace to a maximum z height
		if (wpz > max_obstacle_height_)
		{
			// we know we want the vector's z value to be max_z
			t = std::max(0.0, std::min(t, (max_obstacle_height_ - 0.01 - oz) / c));
		}
		// and we can only raytrace down to the floor
		else if (wpz < origin_z_)
		{
			// we know we want the vector's z value to be 0.0
			t = std::min(t, (origin_z_ - oz) / c);
		}

		// the minimum value to raytrace from is the origin
		if (wpx < origin_x_)
		{
			t = std::min(t, (origin_x_ - ox) / a);
		}
		if (wpy < origin_y_)
		{
			t = std::min(t, (origin_y_ - oy) / b);
		}

		// the maximum value to raytrace to is the end of the map
		if (wpx > map_end_x)
		{
			t = std::min(t, (map_end_x - ox) / a);
		}
		if (wpy > map_end_y)
		{
			t = std::min(t, (map_end_y - oy) / b);
		}

		wpx = ox + a * t;
		wpy = oy + b * t;
		wpz = oz + c * t;

		double point_x, point_y, point_z;
		if (worldToMap3DFloat(wpx, wpy, wpz, point_x, point_y, point_z))
		{
			unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);

			// voxel_grid_.markVoxelLine(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z);
			voxel_grid_.clearVoxelLineInMap(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z, costmap_,
					unknown_threshold_, mark_threshold_, FREE_SPACE, NO_INFORMATION,
					cell_raytrace_range);

			updateRaytraceBounds(ox, oy, wpx, wpy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);

			if (publish_clearing_points)
			{
				geometry_msgs::Point32 point;
				point.x = wpx;
				point.y = wpy;
				point.z = wpz;
				clearing_endpoints_.points.push_back(point);
			}
		}
	}

	if (publish_clearing_points)
	{
		clearing_endpoints_.header.frame_id = global_frame_;
		clearing_endpoints_.header.stamp = pcl_conversions::fromPCL(clearing_observation.cloud_->header).stamp;
		clearing_endpoints_.header.seq = clearing_observation.cloud_->header.seq;

		clearing_endpoints_pub_.publish(clearing_endpoints_);
	}
}

void RgbdVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
	// project the new origin into the grid
	int cell_ox, cell_oy;
	cell_ox = int((new_origin_x - origin_x_) / resolution_);
	cell_oy = int((new_origin_y - origin_y_) / resolution_);

	// compute the associated world coordinates for the origin cell
	// beacuase we want to keep things grid-aligned
	double new_grid_ox, new_grid_oy;
	new_grid_ox = origin_x_ + cell_ox * resolution_;
	new_grid_oy = origin_y_ + cell_oy * resolution_;

	// To save casting from unsigned int to int a bunch of times
	int size_x = size_x_;
	int size_y = size_y_;

	// we need to compute the overlap of the new and existing windows
	int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
	lower_left_x = std::min(std::max(cell_ox, 0), size_x);
	lower_left_y = std::min(std::max(cell_oy, 0), size_y);
	upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
	upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

	unsigned int cell_size_x = upper_right_x - lower_left_x;
	unsigned int cell_size_y = upper_right_y - lower_left_y;


	if(keep_history_)
	{
		// we need a map to store the obstacles in the window temporarily
		unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
		unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
		unsigned int* voxel_map = voxel_grid_.getData();

		// copy the local window in the costmap to the local map
		copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
		copyMapRegion(voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x, cell_size_x,
				cell_size_y);

		// we'll reset our maps to unknown space if appropriate
		resetMaps();

		// update the origin with the appropriate world coordinates
		origin_x_ = new_grid_ox;
		origin_y_ = new_grid_oy;

		// compute the starting cell location for copying data back in
		int start_x = lower_left_x - cell_ox;
		int start_y = lower_left_y - cell_oy;

		// now we want to copy the overlapping information back into the map, but in its new location
		copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
		copyMapRegion(local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_, cell_size_x, cell_size_y);

		// make sure to clean up
		delete[] local_map;
		delete[] local_voxel_map;
	}
	else
	{
		origin_x_ = new_grid_ox;
		origin_y_ = new_grid_oy;
		resetMaps();
	}
}

}  // namespace costmap_2d
