/*
 * .cpp
 *
 *  Created on: Apr 12, 2017
 *      Author: tomzhang
 */

#include <costmap_2d/virtual_layer.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/transforms.h>
//#include <tf/exceptions.h>
PLUGINLIB_EXPORT_CLASS(costmap_2d::VirtualLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;
namespace costmap_2d
{

VirtualLayer::VirtualLayer()
{
    costmap_ =NULL;
    wall_buffer_.reset();


}

VirtualLayer::~VirtualLayer()
{
    if (dsrv_)
        delete dsrv_;
    wall_buffer_.reset();
}
void VirtualLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_), g_nh;
    rolling_window_ = layered_costmap_->isRolling();
    std::string wall_topic;
    bool track_unknown_space;
    nh.param("wall_topic", wall_topic, std::string("virtual_wall"));
    nh.param("global_wall", global_wall_, false);
    nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
    if (true)
        default_value_ = FREE_SPACE;
    current_ = true;
    global_frame_ = layered_costmap_->getGlobalFrameID();
    ROS_ERROR("global_wall %d, global_frame:%s",global_wall_,global_frame_.c_str());
    VirtualLayer::matchSize();
    ROS_ERROR("bound:%f,%f. %d,%d,%f",this->origin_x_,this->origin_y_,this->size_x_,this->size_y_,this->resolution_);

    wall_sub_ = g_nh.subscribe(wall_topic, 1, &VirtualLayer::pointCloud2Callback, this);
    has_wall_ = false;
    has_updated_data_ = true;



    dsrv_ = NULL;
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &VirtualLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

}


void VirtualLayer::reconfigureCB(GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}

void VirtualLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
    sensor_msgs::PointCloud2 wall_cloud;
    {
        boost::mutex::scoped_lock lock(mtx);
        if (!enabled_)
            return;

        if (!has_wall_ )
            return;

        if(global_wall_)
        {
            if(!new_wall_received_)
            {
                has_updated_data_ = false;
                return;
            }
            new_wall_received_ = false;

        }
        this->resetMaps();

        if(rolling_window_)
        {
            updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);		//ROS_ERROR("origin:%f,%f",this->origin_x_,this->origin_y_);
        }


        useExtraBounds(min_x, min_y, max_x, max_y);

        if(wall_buffer_->data.empty())
        {
            has_updated_data_ = false;
            return;
        }

        if(global_frame_!=wall_frame_)
        {
            try
            {

                wall_buffer_->header.stamp = ros::Time::now();
                if(tf_->waitForTransform(global_frame_,wall_frame_ , ros::Time::now(), ros::Duration(0.1)))
                {
                    pcl_ros::transformPointCloud(global_frame_,*wall_buffer_,wall_cloud,*tf_);
                }
                else
                {
                    ROS_WARN("Time Out to wait Frame %s to Frame %s",wall_frame_.c_str(),global_frame_.c_str());
                    return;
                }


            }
            catch (tf::TransformException& ex)
            {
                ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", global_frame_.c_str(),
                          wall_frame_.c_str(), ex.what());
                return;
            }

        }
        else
        {
            wall_cloud = *wall_buffer_;
        }
    }


    sensor_msgs::PointCloud2ConstIterator<float> iter_x(wall_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(wall_cloud, "y");

    double px, py;
    size_t num_points =wall_cloud.width;
    for (size_t i = 0 ;i < num_points; ++i, ++iter_x, ++iter_y)
    {
        px = *iter_x,py = *iter_y;
        // now we need to compute the map coordinates for the observation
        unsigned int mx, my;
        if (!worldToMap(px, py, mx, my))
        {
            //ROS_DEBUG("Computing map coords failed");
            continue;
        }
        unsigned int index = getIndex(mx, my);
        //ROS_ERROR("%d, index:%d",(int)i, index);
        costmap_[index] = LETHAL_OBSTACLE;
        touch(px, py, min_x, min_y, max_x, max_y);
    }

    has_updated_data_ = false;

}

void VirtualLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i,
                               int max_j)
{
    //ROS_INFO("obstaclelayer updateCosts,combination_method_:%d",combination_method_);
    if (!has_wall_)
        return;
    if (!enabled_)
        return;
    if(!has_updated_data_)
    {
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        has_updated_data_ = true;
    }
}


void VirtualLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message)
{

    // buffer the point cloud
    boost::mutex::scoped_lock lock(mtx);
    wall_frame_ = message->header.frame_id;
    new_wall_received_ = true;
    wall_buffer_ = boost::make_shared<sensor_msgs::PointCloud2>((*message));
    has_wall_ = true;
}

void VirtualLayer::activate()
{
    onInitialize();
}
void VirtualLayer::deactivate()
{
    wall_sub_.shutdown();
    dsrv_->clearCallback();
    delete dsrv_;

}

void VirtualLayer::reset()
{
    deactivate();
    resetMaps();
    activate();
}
} // end namespace
