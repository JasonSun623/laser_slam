/*
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>


#include <stdio.h>
#include <stdlib.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"


#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
//for point_cloud::fromROSMsg
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "localization/localization_state.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/thread/mutex.hpp>
#include "std_srvs/Trigger.h"
#include <nav_msgs/Odometry.h>
//#include <bullet/LinearMath/btMatrix3x3.h>>

boost::mutex scan_callback_mutex;
boost::mutex service_mutex_;


//these should be parameters
// defines how good the match has to be to create a candidate for publishing a pose
double ICP_FITNESS_THRESHOLD = 100.1;// =  0.025;
//defines how much distance deviation from amcl to icp pose is needed to make us publish a pose
double DIST_THRESHOLD = 0.05;
// same for angle
double ANGLE_THRESHOLD = 0.01;
double ANGLE_UPPER_THRESHOLD = M_PI / 6;
// accept only scans one second old or younger
double AGE_THRESHOLD = 1;
double UPDATE_AGE_THRESHOLD = 0;

double ICP_INLIER_THRESHOLD = 0.9;
double ICP_INLIER_DIST = 0.1;

double POSE_COVARIANCE_TRANS = 1.5;
double ICP_NUM_ITER = 250;

double SCAN_RATE = 0.2;

double SCAN_LONG_RATE = 0.25;
double SCAN_LONG_THRESHOLD = 1.0;

double icp_first_threshold = 0.9;
double icp_second_threshold = 0.65;
double icp_change_threshold = 0.45;

std::string BASE_LASER_FRAME = "/laser";
std::string ODOM_FRAME = "/odom";

ros::NodeHandle *nh = 0;
ros::Publisher pub_output_;
ros::Publisher pub_output_scan;
ros::Publisher pub_output_scan_transformed;
ros::Publisher pub_info_;

ros::Publisher pub_pose;
ros::Publisher pub_points_number_;
ros::ServiceServer icp_pose_service_;
std_msgs::Float32MultiArray array;
std::string cmd_;
std::string result_;
bool SnapIcp(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
bool isNavState();
ros::ServiceClient status_client_;

laser_geometry::LaserProjection *projector_ = 0;
tf::TransformListener *listener_ = 0;
sensor_msgs::PointCloud2 cloud2;
sensor_msgs::PointCloud2 cloud2transformed;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

boost::shared_ptr< sensor_msgs::PointCloud2> output_cloud = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
boost::shared_ptr< sensor_msgs::PointCloud2> scan_cloud = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());

bool we_have_a_map = false;
bool we_have_a_scan = false;
bool we_have_a_scan_transformed = false;
bool start_scan_2_map_ = true;

bool use_sim_time = true;
double alpha = 0.0;
double lower_limit = 0.55;

int lastScan = 0;
int actScan = 0;
double gary_dist_sqr_ = 0.04*0.04;

/***************************definition by gary**************************/
unsigned int scan_points_total_=0;//一帧激光中原始激光点的总数
unsigned int points_in_map_total=0;//地图中点云的数据量
unsigned int valid_scan_points_total_=0;//把很近的激光点给去除掉，经过滤波之后，有效的激光点总数
unsigned int scan_matched_points_count_=0;//有效激光点与地图之间进行匹配后得到的匹配后的激光点数目
double scan_matched_valid_percentage_=0;//有效匹配的百分比
/*********************************************************************/

bool scan_line_use = false;

double CAMERA_SET_POSE_VALID_DURTIME = 5.0;
int SNAPMAPICP_OPEN = 1;

geometry_msgs::PoseStamped camera_set_pose;
bool camera_set_pose_valid_flag = false;
double CAMERA_SET_POSE_VALID_DISTANCE = 1.2;
nav_msgs::Odometry current_pose;
bool current_pose_rev_flag = false;

/*inline void
  pcl::transformAsMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat)
{
  double mv[12];
  bt.getBasis ().getOpenGLSubMatrix (mv);

  tf::Vector3 origin = bt.getOrigin ();

  out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
  out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
  out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];

  out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
  out_mat (0, 3) = origin.x ();
  out_mat (1, 3) = origin.y ();
  out_mat (2, 3) = origin.z ();
}*/

inline void
matrixAsTransfrom (const Eigen::Matrix4f &out_mat,  tf::Transform& bt)
{
    double mv[12];

    mv[0] = out_mat (0, 0) ;
    mv[4] = out_mat (0, 1);
    mv[8] = out_mat (0, 2);
    mv[1] = out_mat (1, 0) ;
    mv[5] = out_mat (1, 1);
    mv[9] = out_mat (1, 2);
    mv[2] = out_mat (2, 0) ;
    mv[6] = out_mat (2, 1);
    mv[10] = out_mat (2, 2);

    tf::Matrix3x3 basis;
    basis.setFromOpenGLSubMatrix(mv);
    tf::Vector3 origin(out_mat (0, 3),out_mat (1, 3),out_mat (2, 3));

    ROS_DEBUG("origin %f %f %f", origin.x(), origin.y(), origin.z());

    bt = tf::Transform(basis,origin);
}


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_xyz;

pcl::KdTree<pcl::PointXYZ>::Ptr mapTree;


pcl::KdTree<pcl::PointXYZ>::Ptr getTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb)
{
    pcl::KdTree<pcl::PointXYZ>::Ptr tree;
    tree.reset (new pcl::KdTreeFLANN<pcl::PointXYZ>);

    tree->setInputCloud (cloudb);
    return tree;
}


void mapCallback(const nav_msgs::OccupancyGrid& msg)
{   
   	// if(!isNavState())
	// {
	// 	ROS_INFO("robot is not in navgation state now!");
	// 	return;
	// }
    ROS_INFO("I heard frame_id: [%s]", msg.header.frame_id.c_str());

    float resolution = msg.info.resolution;
    float width = msg.info.width;
    float height = msg.info.height;

    float posx = msg.info.origin.position.x;
    float posy = msg.info.origin.position.y;

    cloud_xyz = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());

    //cloud_xyz->width    = 100; // 100
    cloud_xyz->height   = 1;
    cloud_xyz->is_dense = false;
    std_msgs::Header header;
    header.stamp = ros::Time(0.0);
    header.frame_id = "map";

    cloud_xyz->header = pcl_conversions::toPCL(header);

    pcl::PointXYZ point_xyz;

    //for (unsigned int i = 0; i < cloud_xyz->width ; i++)
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            //@TODO
            if (msg.data[x + y * width] == 100)
            {
                point_xyz.x = (.5f + x) * resolution + posx;
                point_xyz.y = (.5f + y) * resolution + posy;
                point_xyz.z = 0;
                cloud_xyz->points.push_back(point_xyz);
            }
        }
    cloud_xyz->width = cloud_xyz->points.size();

    mapTree = getTree(cloud_xyz);

    pcl::toROSMsg (*cloud_xyz, *output_cloud);
    ROS_INFO("Publishing PointXYZ cloud with %ld points in frame %s", cloud_xyz->points.size(),output_cloud->header.frame_id.c_str());

    we_have_a_map = true;
}


int lastTimeSent = -1000;

int count_sc_ = 0;

bool getTransform(tf::StampedTransform &trans , const std::string parent_frame, const std::string child_frame, const ros::Time stamp)
{   ROS_DEBUG("getTransform_start");
    bool gotTransform = false;

    ros::Time before = ros::Time::now();
    if (!listener_->waitForTransform(parent_frame, child_frame, stamp, ros::Duration(0.5)))
    {
        ROS_WARN("DIDNT GET TRANSFORM %s %s IN c at %f", parent_frame.c_str(), child_frame.c_str(), stamp.toSec());
        return false;
    }
    //ROS_INFO("waited for transform %f", (ros::Time::now() - before).toSec());

    try
    {
        gotTransform = true;
        listener_->lookupTransform(parent_frame,child_frame,stamp , trans);
    }
    catch (tf::TransformException ex)
    {
        gotTransform = false;
        ROS_WARN("DIDNT GET TRANSFORM %s %s IN B", parent_frame.c_str(), child_frame.c_str());
    }


    return gotTransform;
}

ros::Time last_processed_scan;


bool isNavState()
{
	localization::localization_state srv;
	status_client_.call(srv);
	ROS_DEBUG("slam_state :%s",srv.response.state.c_str());
	return (0 == srv.response.state.compare("amcl"));
}

bool scanHandle(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    //ROS_INFO("scanCallback");
    
    //if(!start_scan_2_map_)
    //{
    // ros::Rate loop_rate(1);
    //  ROS_INFO("waiting for service cmd");
    //    loop_rate.sleep();
    //return;
    //}
    //  if(!isNavState())
	// {
	// 	ROS_INFO("robot is not in navgation state now!");
	// 	return;
	// }
    if(1 == SNAPMAPICP_OPEN)
        ROS_DEBUG("snapmapicp open");

    bool rlt = false;
    if (!we_have_a_map)
    {
        ROS_INFO("SnapMapICP waiting for map to be published");
        result_ = "SnapMapICP waiting for map to be published";
        //service_mutex_.unlock();
        return false;
    }

    ros::Time scan_in_time = scan_in->header.stamp;
    ros::Time time_received = ros::Time::now();
     if ( scan_in_time - last_processed_scan < ros::Duration(1.0f / SCAN_RATE) )
    {
        //ROS_INFO("rejected scan, last %f , this %f", last_processed_scan.toSec() ,scan_in_time.toSec());
        return false;
    }/**/


    //projector_.transformLaserScanToPointCloud("base_link",*scan_in,cloud,listener_);
    if (!scan_callback_mutex.try_lock())
	{
		ROS_INFO("try lock failed");        
		//service_mutex_.unlock();
        return false;
	}
    ros::Duration scan_age = ros::Time::now() - scan_in_time;

    //check if we want to accept this scan, if its older than 1 sec, drop it
    if (!use_sim_time)
    {
        if (scan_age.toSec() > AGE_THRESHOLD)
        {
            //ROS_WARN("SCAN SEEMS TOO OLD (%f seconds, %f threshold)", scan_age.toSec(), AGE_THRESHOLD);
            ROS_WARN("SCAN SEEMS TOO OLD (%f seconds, %f threshold) scan time: %f , now %f", scan_age.toSec(), AGE_THRESHOLD, scan_in_time.toSec(),ros::Time::now().toSec() );
            //service_mutex_.unlock();
            scan_callback_mutex.unlock();

            return false;
        }
    }

    count_sc_++;
    //ROS_INFO("count_sc %i MUTEX LOCKED", count_sc_);

    //if (count_sc_ > 10)
    //if (count_sc_ > 10)
    {
        count_sc_ = 0;

        tf::StampedTransform base_at_laser;
        if (!getTransform(base_at_laser, ODOM_FRAME, "base_link", scan_in_time))
        {
            ROS_WARN("Did not get base pose at laser scan time");
            result_ = "Did not get base pose at laser scan time";
            //service_mutex_.unlock();
            scan_callback_mutex.unlock();

            return false;
        }


        sensor_msgs::PointCloud cloud;
        sensor_msgs::PointCloud cloudInMap;

        projector_->projectLaser(*scan_in,cloud);

        we_have_a_scan = false;
        bool gotTransform = false;

        if (!listener_->waitForTransform("/map", cloud.header.frame_id, cloud.header.stamp, ros::Duration(0.05)))
        {
            
            ROS_WARN("SnapMapICP no map to cloud transform found MUTEX UNLOCKED");
            //result_ = "SnapMapICP no map to cloud transform found MUTEX UNLOCKED";
            //service_mutex_.unlock();
            scan_callback_mutex.unlock();
            return false;
        }

        if (!listener_->waitForTransform("/map", "/base_link", cloud.header.stamp, ros::Duration(0.05)))
        {
            
            ROS_WARN("SnapMapICP no map to base transform found MUTEX UNLOCKED");
            //result_ = "SnapMapICP no map to base transform found MUTEX UNLOCKED";
            //service_mutex_.unlock();
            scan_callback_mutex.unlock();
            return false;
        }


        while (!gotTransform && (ros::ok()))
        {
            try
            {
                gotTransform = true;
                listener_->transformPointCloud ("/map",cloud,cloudInMap);
            }
            catch (...)
            {
                gotTransform = false;
                ROS_WARN("DIDNT GET TRANSFORM IN A");
            }
        }

        for (size_t k =0; k < cloudInMap.points.size(); k++)
        {
            cloudInMap.points[k].z = 0;
        }


        gotTransform = false;
        tf::StampedTransform oldPose;
        while (!gotTransform && (ros::ok()))
        {
            try
            {
                gotTransform = true;
                listener_->lookupTransform("/map", "/base_link",
                                           cloud.header.stamp , oldPose);
            }
            catch (tf::TransformException ex)
            {
                gotTransform = false;
                ROS_WARN("DIDNT GET TRANSFORM IN B");
            }
        }
        if (we_have_a_map && gotTransform)
        {
            sensor_msgs::convertPointCloudToPointCloud2(cloudInMap,cloud2);
            we_have_a_scan = true;

            actScan++;

            //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> reg;
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
            reg.setTransformationEpsilon (1e-6);
            // Set the maximum distance between two correspondences (src<->tgt) to 10cm
            // Note: adjust this based on the size of your datasets
            reg.setMaxCorrespondenceDistance(1.5);
            reg.setMaximumIterations (ICP_NUM_ITER);
            // Set the point representation

            //ros::Time bef = ros::Time::now();

            PointCloudT::Ptr myMapCloud (new PointCloudT());
            PointCloudT::Ptr myScanCloud (new PointCloudT());

            pcl::fromROSMsg(*output_cloud,*myMapCloud);
            pcl::fromROSMsg(cloud2,*myScanCloud);

            reg.setInputSource(myScanCloud);
            reg.setInputTarget(myMapCloud);


            //统计激光的点云数据量
            scan_points_total_=myScanCloud->size();
            //统计地图中的点云的数据量
            points_in_map_total=myMapCloud->size();
            //定义在激光中点的坐标
            pcl::PointXYZ point_in_scan;
            //定义在map中点的坐标
            pcl::PointXYZ point_in_map;
            //设置一个标志位表示是否找到该点
            bool find=false;
            for(int i=0;i<scan_points_total_;i++)
            {
                point_in_scan=myScanCloud->at(i);
                for(int j=0;j<points_in_map_total;j++)
                {
                    point_in_map=myMapCloud->at(j);
                    //if(((point_in_scan.x-point_in_map.x)*(point_in_scan.x-point_in_map.x)+(point_in_scan.y-point_in_map.y)*(point_in_scan.y-point_in_map.y))< gary_dist_sqr_)
                    if(sqrt((point_in_scan.x-point_in_map.x)*(point_in_scan.x-point_in_map.x)+(point_in_scan.y-point_in_map.y)*(point_in_scan.y-point_in_map.y))< 0.042)
                    {
                        find=true;
                        break;
                    }
                }
                if(find==true)
                {
                    scan_matched_points_count_++;
                    find = false;
                }
            }

            scan_matched_valid_percentage_=(double)(scan_matched_points_count_) /( scan_points_total_);
            //ROS_ERROR("scan_matched_points_count_:%d,scan_points_total_:%d",scan_matched_points_count_,scan_points_total_);
            scan_matched_points_count_=0;
            
        //     //ROS_INFO("valid percentage is %f",scan_matched_valid_percentage_);

        //     /************************************************************/



            PointCloudT unused;
            int i = 0;

            reg.align (unused);

            const Eigen::Matrix4f &transf = reg.getFinalTransformation();
            tf::Transform t;
            matrixAsTransfrom(transf,t);

            //ROS_ERROR("proc time %f", (ros::Time::now() - bef).toSec());

            we_have_a_scan_transformed = false;
            PointCloudT transformedCloud;
            pcl::transformPointCloud (*myScanCloud, transformedCloud, reg.getFinalTransformation());

            double inlier_perc = 0;
            {
                // count inliers
                std::vector<int> nn_indices (1);
                std::vector<float> nn_sqr_dists (1);

                size_t numinliers = 0;

                for (size_t k = 0; k < transformedCloud.points.size(); ++k )
                {
                    if (mapTree->radiusSearch (transformedCloud.points[k], ICP_INLIER_DIST, nn_indices,nn_sqr_dists, 1) != 0)
                        numinliers += 1;
                }
                if (transformedCloud.points.size() > 0)
                {
                    //ROS_INFO("Inliers in dist %f: %zu of %zu percentage %f (%f)", ICP_INLIER_DIST, numinliers, transformedCloud.points.size(), (double) numinliers / (double) transformedCloud.points.size(), ICP_INLIER_THRESHOLD);
                    inlier_perc = (double) numinliers / (double) transformedCloud.points.size();
                }
                //Clear array
                array.data.clear();
                array.data.push_back(numinliers);
                array.data.push_back(myScanCloud->size());
                array.data.push_back(inlier_perc);
                array.data.push_back(scan_matched_valid_percentage_);
                pub_points_number_.publish(array);
            }

            last_processed_scan = scan_in_time;

            pcl::toROSMsg (transformedCloud, cloud2transformed);
            we_have_a_scan_transformed = true;

            double dist = sqrt((t.getOrigin().x() * t.getOrigin().x()) + (t.getOrigin().y() * t.getOrigin().y()));
            double angleDist = t.getRotation().getAngle();
            tf::Vector3 rotAxis  = t.getRotation().getAxis();
            t =  t * oldPose;

            tf::StampedTransform base_after_icp;
            if (!getTransform(base_after_icp, ODOM_FRAME, "base_link", ros::Time(0)))
            {
                ROS_WARN("Did not get base pose at now");
                //service_mutex_.unlock();
                scan_callback_mutex.unlock();
                return false;
            }
            else
            {
                tf::Transform rel = base_at_laser.inverseTimes(base_after_icp);
                ROS_DEBUG("relative motion of robot while doing icp: %fcm %fdeg", rel.getOrigin().length(), rel.getRotation().getAngle() * 180 / M_PI);
                t= t * rel;
            }

            //ROS_INFO("dist %f angleDist %f",dist, angleDist);

            //ROS_INFO("SCAN_AGE seems to be %f", scan_age.toSec());
            char msg_c_str[2048];
            sprintf(msg_c_str,"INLIERS %f (%f) scan_age %f (%f age_threshold) dist %f angleDist %f axis(%f %f %f) fitting %f (icp_fitness_threshold %f)",inlier_perc, ICP_INLIER_THRESHOLD, scan_age.toSec(), AGE_THRESHOLD ,dist, angleDist, rotAxis.x(), rotAxis.y(), rotAxis.z(),reg.getFitnessScore(), ICP_FITNESS_THRESHOLD );
            std_msgs::String strmsg;
            strmsg.data = msg_c_str;

            //ROS_INFO("%s", msg_c_str);
            //ROS_ERROR("reg.getFitnessScore():%f",reg.getFitnessScore());
            double cov = POSE_COVARIANCE_TRANS;

            //if ((actScan - lastTimeSent > UPDATE_AGE_THRESHOLD) && ((dist > DIST_THRESHOLD) || (angleDist > ANGLE_THRESHOLD)) && (angleDist < ANGLE_UPPER_THRESHOLD))
            //  if ( reg.getFitnessScore()  <= ICP_FITNESS_THRESHOLD )
	    	    //std::cerr << "actScan - lastTimeSent: " << actScan - lastTimeSent << " " << "dist: " << dist << " " << "angleDist: " << angleDist << " inlier_perc: " << inlier_perc << std::endl;
            // if ((actScan - lastTimeSent > UPDATE_AGE_THRESHOLD) && ((dist > DIST_THRESHOLD) || (angleDist > ANGLE_THRESHOLD)) 
            // && (inlier_perc > ICP_INLIER_THRESHOLD) && (angleDist < ANGLE_UPPER_THRESHOLD))
            int scan_long_count = 0;
            for(int j=0; j<scan_in->ranges.size();j++)
            {
                if(scan_in->ranges[j] > SCAN_LONG_THRESHOLD)
                {
                    scan_long_count++;
                }
            }
            double scan_long_rate = (double)scan_long_count/scan_in->ranges.size();

            ROS_DEBUG("dist:%f,angledist:%f",dist,angleDist);
            if ((actScan - lastTimeSent >= UPDATE_AGE_THRESHOLD) && 
                ((dist > DIST_THRESHOLD) || (angleDist > ANGLE_THRESHOLD)) && 
                (angleDist < ANGLE_UPPER_THRESHOLD) &&
                (scan_long_rate > SCAN_LONG_RATE) &&
                (!camera_set_pose_valid_flag) &&
                (1 == SNAPMAPICP_OPEN))
            {
                ROS_DEBUG("prepare to setup initialpose,inlier_perc:%f,or:%f",inlier_perc,scan_matched_valid_percentage_);
                lastTimeSent = actScan;
                geometry_msgs::PoseWithCovarianceStamped pose;
                pose.header.frame_id = "map";
                pose.pose.pose.position.x = t.getOrigin().x();
                pose.pose.pose.position.y = t.getOrigin().y();

                tf::Quaternion quat = t.getRotation();
                //quat.setRPY(0.0, 0.0, theta);
                tf::quaternionTFToMsg(quat,pose.pose.pose.orientation);
                float factorPos = 0.03;
                float factorRot = 0.1;
                float factorPos_large = 0.12;
                float factorRot_large = 0.2;
                if((inlier_perc > ICP_INLIER_THRESHOLD))
                {
                    ROS_DEBUG("small cov publish");
                    pose.pose.covariance[6*0+0] = (cov * cov) * factorPos;
                    pose.pose.covariance[6*1+1] = (cov * cov) * factorPos;
                    pose.pose.covariance[6*3+3] = (M_PI/12.0 * M_PI/12.0) * factorRot;
                }
                else if ((inlier_perc < ICP_INLIER_THRESHOLD)  && (icp_second_threshold < inlier_perc))
                {
                    ROS_DEBUG("large cov publish");
                    if(inlier_perc - scan_matched_valid_percentage_ > icp_change_threshold)
                    {
                        pose.pose.covariance[6*0+0] = (cov * cov) * factorPos_large;
                        pose.pose.covariance[6*1+1] = (cov * cov) * factorPos_large;
                        pose.pose.covariance[6*3+3] = (M_PI/12.0 * M_PI/12.0) * factorRot_large;
                    }
                    else
                    {
                        ROS_DEBUG("not become better");
                        scan_callback_mutex.unlock();
                        scan_line_use = false;
                        return false;
                    }
                }
                else
                {
                    ROS_DEBUG("low");
                    scan_callback_mutex.unlock();
                    scan_line_use = false;
                    return false;
                }

                ROS_DEBUG("i %i converged %i SCORE: %f", i,  reg.hasConverged (),  reg.getFitnessScore()  );
                ROS_DEBUG("PUBLISHING A NEW INITIAL POSE dist %f angleDist %f Setting pose: %.3f %.3f  [frame=%s]",dist, angleDist , pose.pose.pose.position.x  , pose.pose.pose.position.y , pose.header.frame_id.c_str());
                pub_pose.publish(pose);
                strmsg.data += " << SENT";
                result_ = "Publish succeed";
                scan_line_use = true; 
                rlt = true;
                //ros::Rate loop_rate(0.1);
                //loop_rate.sleep();
            }
            else
            {
                result_ = "Publish failed";
                rlt = false;
            }

            //ROS_INFO("processing time : %f", (ros::Time::now() - time_received).toSec());

            pub_info_.publish(strmsg);
            //ROS_INFO("map width %i height %i size %i, %s", myMapCloud.width, myMapCloud.height, (int)myMapCloud.points.size(), myMapCloud.header.frame_id.c_str());
            //ROS_INFO("scan width %i height %i size %i, %s", myScanCloud.width, myScanCloud.height, (int)myScanCloud.points.size(), myScanCloud.header.frame_id.c_str());
        }
    }
    //service_mutex_.unlock();
    scan_callback_mutex.unlock();
    return rlt;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(!scan_line_use)
    {
        ROS_DEBUG("snapmapicp scan used");
        scanHandle(scan_in);
    }
}

void scanLinesCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(scan_line_use)
    {
        ROS_DEBUG("snapmapicp scan line used");
        scanHandle(scan_in);
    }    
}

bool SnapIcp(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    ROS_INFO("SnapICP service");
    //cmd_ = req.cmd;
    start_scan_2_map_ = true;
    //boost::mutex service_mutex_;
    if (!service_mutex_.try_lock())
    {
        res.message = "lock failed";
        res.success = false;
    }
    res.message = result_;
    res.success = true;
    start_scan_2_map_ = false;
    return true;
}

ros::Time paramsWereUpdated;

void cameraSetPoseRecordCallback(const geometry_msgs::PoseStamped& msg)
{
    camera_set_pose = msg;
    camera_set_pose_valid_flag = true;
}

void currentPoseCallback(const nav_msgs::Odometry msg)
{
    current_pose = msg;
    current_pose_rev_flag = true;
}

void updateParams()
{
    paramsWereUpdated = ros::Time::now();
    // nh.param<std::string>("default_param", default_param, "default_value");
    nh->param<bool>("/snap_map_icp/USE_SIM_TIME", use_sim_time, true);
    nh->param<double>("/snap_map_icp/icp_fitness_threshold", ICP_FITNESS_THRESHOLD, 100 );
    nh->param<double>("/snap_map_icp/age_threshold", AGE_THRESHOLD, 1);
    nh->param<double>("/snap_map_icp/angle_upper_threshold", ANGLE_UPPER_THRESHOLD, 1);
    nh->param<double>("/snap_map_icp/angle_threshold", ANGLE_THRESHOLD, 0.01);
    nh->param<double>("/snap_map_icp/update_age_threshold", UPDATE_AGE_THRESHOLD, 0);
    nh->param<double>("/snap_map_icp/dist_threshold", DIST_THRESHOLD, 0.01);
    nh->param<double>("/snap_map_icp/icp_inlier_threshold", ICP_INLIER_THRESHOLD, 0.80);
    nh->param<double>("/snap_map_icp/icp_inlier_dist", ICP_INLIER_DIST, 0.1);
    nh->param<double>("/snap_map_icp/icp_num_iter", ICP_NUM_ITER, 250);
    nh->param<double>("/snap_map_icp/pose_covariance_trans", POSE_COVARIANCE_TRANS, 0.5);
    nh->param<double>("/snap_map_icp/scan_rate", SCAN_RATE, 0.2);
    nh->param<double>("/snap_map_icp/camera_set_pose_valid_durtime", CAMERA_SET_POSE_VALID_DURTIME, 5.0);
    nh->param<double>("/snap_map_icp/camera_set_pose_valid_distance", CAMERA_SET_POSE_VALID_DISTANCE, 1.2);
    nh->param<int>("/snap_map_icp/snapmapicp_open", SNAPMAPICP_OPEN, 1);
    if (SCAN_RATE < .001)
        SCAN_RATE  = 0.001;
    nh->param<double>("/snap_map_icp/icp_first_threshold", icp_first_threshold, 0.9);
    nh->param<double>("/snap_map_icp/icp_second_threshold", icp_second_threshold, 0.65);
    nh->param<double>("/snap_map_icp/icp_change_threshold", icp_change_threshold, 0.45);
}

int main(int argc, char** argv)
{

// Init the ROS node
    ros::init(argc, argv, "snapmapicp");
    ros::NodeHandle nh_;
    nh = &nh_;

    nh->param<std::string>("/snap_map_icp/odom_frame", ODOM_FRAME, "/odom");
    nh->param<std::string>("/snap_map_icp/base_laser_frame", BASE_LASER_FRAME, "/laser");

    last_processed_scan = ros::Time::now();

    projector_ = new laser_geometry::LaserProjection();
    tf::TransformListener listener;
    listener_ = &listener;

    pub_points_number_ = nh->advertise<std_msgs::Float32MultiArray>("points_number", 1);
    pub_info_ =  nh->advertise<std_msgs::String> ("SnapMapICP", 1);
    pub_output_ = nh->advertise<sensor_msgs::PointCloud2> ("map_points", 1);
    pub_output_scan = nh->advertise<sensor_msgs::PointCloud2> ("scan_points", 1);
    pub_output_scan_transformed = nh->advertise<sensor_msgs::PointCloud2> ("scan_points_transformed", 1);
    pub_pose = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    icp_pose_service_ = nh->advertiseService("pub_icp_pose", &SnapIcp);


    ros::Subscriber subMap = nh_.subscribe("map", 1, mapCallback);
    ros::Subscriber subScan = nh_.subscribe("scan", 1, scanCallback);
    ros::Subscriber subScanLines = nh_.subscribe("scan_lines", 1, scanLinesCallback);

    ros::Subscriber subCameraSetPose = nh_.subscribe("camera_setpose_record", 1, cameraSetPoseRecordCallback);
    ros::Subscriber currentPose = nh_.subscribe("current_pose",1, currentPoseCallback);
    ros::Rate loop_rate(3);

    listener_->waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(5));

    listener_->waitForTransform(BASE_LASER_FRAME, "/map", ros::Time(0), ros::Duration(5));

    ros::AsyncSpinner spinner(2);
    spinner.start();

    updateParams();

    ROS_ERROR("SnapMapICP running.");
    double camera_current_distance = 0.0;

    while (ros::ok())
    {   //ROS_DEBUG("while (ros::ok())");
        //ROS_DEBUG("actScan:%d,lastScan:%d",actScan,lastScan);
        if (actScan > lastScan)
        {   
            ROS_DEBUG("actScan > lastScan");
            lastScan = actScan;
            // publish map as a pointcloud2
            if (we_have_a_map)
              pub_output_.publish(output_cloud);
            // publish scan as seen as a pointcloud2
            if (we_have_a_scan)
               pub_output_scan.publish(cloud2);
            // publish icp transformed scan
            if (we_have_a_scan_transformed)
                pub_output_scan_transformed.publish(cloud2transformed);
        }
        
        /*if(camera_set_pose_valid_flag &&
            ros::Time::now() - camera_set_pose.header.stamp > ros::Duration(CAMERA_SET_POSE_VALID_DURTIME))
        {
            camera_set_pose_valid_flag = false;
            ROS_INFO("snapmapicp camera set pose invalid");
        }*/

        if(camera_set_pose_valid_flag && current_pose_rev_flag)
        {
            camera_current_distance = hypot((current_pose.pose.pose.position.x - camera_set_pose.pose.position.x),
                                    (current_pose.pose.pose.position.y - camera_set_pose.pose.position.y));
            ROS_INFO("snapmapicp camera pose & current pose distance%.3f",camera_current_distance);
            if(camera_current_distance > CAMERA_SET_POSE_VALID_DISTANCE)
            {
                camera_set_pose_valid_flag = false;
                ROS_INFO("snapmapicp camera set pose invalid"); 
            }
            current_pose_rev_flag = false;
        }

        if (ros::Time::now() - paramsWereUpdated > ros::Duration(1))
            updateParams();

        loop_rate.sleep();
        ros::spinOnce();
    }

}
