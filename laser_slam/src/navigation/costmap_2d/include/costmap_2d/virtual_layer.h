/*
 * auto_charging_node.h
 *
 *  Created on: Apr 11, 2017
 *      Author: tomzhang
 */
#ifndef INCLUDE_VIRTUAL_LAYER_VIRTUAL_LAYER_H_
#define INCLUDE_VIRTUAL_LAYER_VIRTUAL_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/observation_buffer.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/mutex.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>



namespace costmap_2d
{

class VirtualLayer : public CostmapLayer
{
public:
    VirtualLayer();
    ~VirtualLayer();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y);
    virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual void activate();
    virtual void deactivate();
    virtual void reset();
    void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message);
protected:

    void reconfigureCB(GenericPluginConfig &config, uint32_t level);

    bool rolling_window_;
    bool use_maximum_;
    std::string global_frame_,wall_frame_;
    ros::Subscriber wall_sub_;
    boost::mutex mtx;
    boost::shared_ptr<sensor_msgs::PointCloud2> wall_buffer_;
    dynamic_reconfigure::Server<GenericPluginConfig> *dsrv_;
    unsigned int x_, y_, width_, height_;
    bool has_wall_ ,has_updated_data_,global_wall_, new_wall_received_;
};//end class

}//end namespace
#endif
