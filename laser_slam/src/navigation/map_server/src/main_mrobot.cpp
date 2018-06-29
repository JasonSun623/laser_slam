/*
 * main_mrobot_op.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: tomzhang
 */

#include "map_server/map_server_mrobot.h"
#include "convert/PGM2PNG.h"
#include "convert/PGM2PNGRequest.h"
#include "convert/PGM2PNGResponse.h"
#include "map_server/map_server_functions.hpp"
#include "map_server/virtual_wall_function.hpp"
#include "map_server/region_operations.hpp"
#include "map_server/fixpath_operations.hpp"
#include "map_server/label_function.hpp"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h" 
#include "tf/transform_listener.h"  
#include "tf/message_filter.h"  
#include "tf/tf.h"
#include <ar_track_alvar_msgs/AlvarId.h>
#include <algorithm>
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>
#include "localization/localization_state.h"
#include "tf/transform_datatypes.h"
#include <boost/tuple/tuple.hpp>


#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
		"  map.yaml: map description file\n" \
		"DEPRECATED USAGE: map_server <map> <resolution>\n" \
		"  map: image file to load\n"\
		"  resolution: map resolution [meters/pixel]"

using namespace map_server;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
	std::string path;
	if (argc != 3 && argc != 2)
	{
		ROS_ERROR("%s", USAGE);
		//char   buffer[255];
		//getcwd(buffer, 255); // By using this cmd the working directory of exc can be found
		path = "/home/robot/install/share/map_server/maps/";
		ROS_ERROR("Using default map path %s",path.c_str());
	}
	else
	{
		path=argv[1];
	}

	if (argc != 2)
	{
		ROS_WARN("Using deprecated map server interface. Please switch to new interface.");
	}

	if (-1 == access(path.c_str(), F_OK))
	{
		if (-1 == mkdir((path).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
		{
			ROS_ERROR("Failed to access path %s",path.c_str());
		}
	}
	double res = (argc == 2) ? 0.0 : atof(argv[2]);

	try
	{
		MapServer ms(path, res);
		ros::spin();
	} catch (std::runtime_error& e)
	{
		ROS_ERROR("map_server exception: %s", e.what());
		return -1;
	}


	return 0;
}

MapServer::MapServer(const std::string path, double res) :
									private_nh_("~"), map_loaded(false), res_(res),start_edit_map_(false), deprecated(res != 0),config_map_path_(path),
									thread_stop_(true),rmgr_(NULL),record_path_start(false)
{	
	last_rcv_id_ = -1;
	tf_listener = new tf::TransformListener();
	metadata_pub = n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1,true);
	// Latched publisher for data
	map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	map_filter_pub_ = n_.advertise<gmapping::map_filter>("map_filter", 1, true);
	// communicate with app
	app_pub_ = n_.advertise<std_msgs::String>("app_sub", 10);
	//fb region param cmd from app
	app_param_pub_ = n_.advertise<std_msgs::String>("app_param_sub", 10);
	//sub region param cmd from app
	app_param_sub_ = n_.subscribe("app_param_pub", 10,&MapServer::jsonRegionCallback, this);
	// pub map list
	map_list_pub_ = n_.advertise<std_msgs::String>("map_list_pub", 1);
	// pub label
	app_label_pub_ = n_.advertise<std_msgs::String>("app_label_sub", 1);
	// receive order from app
	app_sub_ = n_.subscribe("app_pub", 10,&MapServer::jsonCallback, this);
	// receive map data from gmapping
	map_sub_ = n_.subscribe("map_build", 1, &MapServer::mapSaverCallback,this);
	// receive map data from app
	app_map_sub_ = n_.subscribe("map_edit", 1, &MapServer::mapEditCallback,this);
	// receive camera label data
	label_sub_ = n_.subscribe("star_info", 1, &MapServer::labelCallback,this);
	//service of converting pgm to png.
    convert_client_ = n_.serviceClient<convert::PGM2PNG>("convert_pgm_2_png");
	// init map_list_manager point
	map_list_manager_ = boost::shared_ptr<map_server::MapListManager>(new map_server::MapListManager(path));
	// pub virtual wall to costmap
	virtual_wall_pub = n_.advertise<sensor_msgs::PointCloud2>("virtual_wall", 1,true);
	// register self state to robot state keeper
	register_client_ = n_.serviceClient<robot_state_keeper::RegisterState>(
			"/robot_state_keeper/register_robot_modules");
	// publish the current pose obtaining by label
	initial_pose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10, true);
	query_uwb_service = private_nh_.advertiseService("query_uwb_points", &MapServer::queryUwbPoints, this);
	// publish all the points belonging to the request scene
	query_all_points_service = private_nh_.advertiseService("query_all_points",&MapServer::queryAllPoints, this);
	
	yaml_pose_pub_ = n_.advertise<nav_msgs::Odometry>("yamlpose", 10, true);
	camera_setpose_record_ = n_.advertise<geometry_msgs::PoseStamped>("camera_setpose_record", 1, true);
	current_pose_sub = n_.subscribe("/current_pose", 10, &MapServer::current_pose_callback, this);

	status_client_ = n_.serviceClient<localization::localization_state>("/localization/get_localizaiton_state");
	start_pose_check = false;
	ros::param::set("start_pose_check",start_pose_check);
	this->rmgr_ = new RegionManager(path);
	// init functions map, all cmd from app will be store here, and find by cmd name
	this->initFuncsMap();
	// int cmd processor object,the object praser the input message to json and it as a queue store and pop the task
	this->initCMDProcessor();
	// manage the thread which handle the task in queue and the thread register the state of map server
	// default thread num = 2;
	this->initCMDThreadPool();
	this->private_nh_.param("step",step_,0.025);
	//ROS_ERROR("step is %f",step_);
	this->private_nh_.param("auto_label_localization",auto_label_localization_,false);
	this->private_nh_.param("auto_label_save",auto_label_save_,false);
	this->private_nh_.param("reset_label_duration",reset_label_duration_double_,240.0);
	this->private_nh_.param("distance_threshold_max",distance_threshold_max,10.0);
	this->private_nh_.param("angle_threshold_max",angle_threshold_max,M_PI_2);
	this->private_nh_.param("distance_threshold_min",distance_threshold_min,0.1);
	this->private_nh_.param("angle_threshold_min",angle_threshold_min,0.05);

	this->private_nh_.param("label_initial_pose_covx",label_initial_pose_covx,0.25);
	this->private_nh_.param("label_initial_pose_covy",label_initial_pose_covy,0.25);
	this->private_nh_.param("label_initial_pose_covth",label_initial_pose_covth,0.0685);

	//ros::Duration reset_label_duration_(reset_label_duration_double_);
	//ROS_ERROR("auto_label_save_:%d",auto_label_save_);

	ros::NodeHandle nh("fixpath_controller");
	fixpath_plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);
	get_fixpath_srv = nh.advertiseService("get_fixpath_point",&MapServer::get_fixpath_service,this);
    //query_all_points_client = nh.serviceClient<map_server::query_all_points>("/map_server_mrobot/query_all_points");
    //nh.param("maps_path", maps_path, std::string("/home/robot/catkin_ws/src/navigation/map_server/maps/"));
}

MapServer::~MapServer()
{
	if(rmgr_ == NULL)
	{
		delete(rmgr_);
	}
	thread_stop_ = true;
	threadpool_.interrupt_all();
}

void MapServer::jsonCallback(const std_msgs::String::ConstPtr msg)
{
	std::string app_cmd;
	Json::Value root_in;
	if(!cmd_processor_->praserStringToJson(msg->data,root_in))
	{
		return;
	}
	Jptr jptr(new Json::Value(root_in));
	cmd_processor_->extractCMD(app_cmd, jptr);
	FuncMap::iterator it;
	if((it=function_map_ptr_->find(app_cmd))!=function_map_ptr_->end())
	{
		{
			boost::mutex::scoped_lock lock(task_mtx_);

			cmd_processor_->pushTask(boost::bind(&MapServer::taskRunner,this,it->second,jptr,app_cmd,app_pub_));
		}
		cond_pull_.notify_one();
	}


}
void MapServer::jsonRegionCallback(const std_msgs::String::ConstPtr msg)
{
	std::string app_cmd;
	Json::Value root_in;
	if(!cmd_processor_->praserStringToJson(msg->data,root_in))
	{
		return;
	}
	Jptr jptr(new Json::Value(root_in));
	cmd_processor_->extractCMD(app_cmd, jptr);
	FuncMap::iterator it;
	if((it=function_region_ptr_->find(app_cmd))!=function_region_ptr_->end())
	{
		{
			boost::mutex::scoped_lock lock(task_mtx_);

			cmd_processor_->pushTask(boost::bind(&MapServer::taskRunner,this,it->second,jptr,app_cmd,app_param_pub_));
		}
		cond_pull_.notify_one();
	}


}
void MapServer::mapSaverCallback(const nav_msgs::OccupancyGridConstPtr map)
{

	
		//lock the common variable
		{
			boost::mutex::scoped_lock lock(save_map_mtx_);
			save_map_ptr_ = map;
		}
}
void MapServer::mapEditCallback(const nav_msgs::OccupancyGridConstPtr &map)
{
    boost::mutex::scoped_lock lock(edit_map_mtx_);
    // check whether this map data should be stored. if yes, we stored it in save_map_ptr_
    if(true == start_edit_map_)
    {
        //lock the common variable

        edit_map_ptr_ = map;
        start_edit_map_ = false;
        lock.unlock();
        // wake up one thread to process map data
        edit_map_cond_.notify_one();
    }
}

void MapServer::current_pose_callback(const nav_msgs::Odometry& msg)
{
	cur_pose.pose = msg.pose.pose;

    if(record_path_start)
    {
        ROS_INFO("path size:%d",(int)(record_path.size()));
        if(0 == record_path.size())
        {
            record_path.push_back(cur_pose);
        }
        else
        {
            double dist_xy = line_dist(record_path[record_path.size()-1].pose.position.x,record_path[record_path.size()-1].pose.position.y,cur_pose.pose.position.x,cur_pose.pose.position.y);
            double dist_th = adjust_th(tf::getYaw(cur_pose.pose.orientation) - tf::getYaw(record_path[record_path.size()-1].pose.orientation));
            ROS_INFO("dist  xy,th:%f,%f",dist_xy,dist_th);
            if(dist_xy > 0.05 || fabs(dist_th) > 0.15)
            {
                ROS_INFO("push a pose");
                record_path.push_back(cur_pose);
            }
        }
    }

    //ROS_INFO("amcl current location %.3f, %.3f", cur_pose.pose.position.x,cur_pose.pose.position.y);
}

bool MapServer::isNavState()
{
	localization::localization_state srv;
	status_client_.call(srv);
	ROS_DEBUG("slam_state :%s",srv.response.state.c_str());
	return (0 == srv.response.state.compare("amcl"));
}

int MapServer::label2Label2Map()
{
	tf::Stamped<tf::Pose> tmp_pose;
	tf::Pose tmp_pose1;
	tf::Quaternion q;
  	//q.setRPY(0, 0, -label_.th);
	//double x,y,z;
	//x = -(cos(-label_.th)*label_.x-sin(-label_.th)*label_.y);
	//y = -(sin(-label_.th)*label_.x+cos(-label_.th)*label_.y);
	//tf::Stamped<tf::Pose> label_pose(tf::Pose(q,tf::Vector3(-x, -y, -label_.z)),ros::Time(0), "usb_cam");
	q.setRPY(0, 0, label_.th);
	tmp_pose1 = tf::Pose(q,tf::Vector3(label_.x, label_.y, -label_.z));
	tf::Stamped<tf::Pose> label_pose(tmp_pose1.inverse(),ros::Time(0), "usb_cam_up");
	//tmp_pose1 = (label_pose).inverse();
	try  
    {  
        tf_listener -> transformPose("map", label_pose, tmp_pose);

		nav_msgs::Odometry odom;
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "map";
		tf::poseTFToMsg(tmp_pose,odom.pose.pose);
		//odom.child_frame_id = "label";
		odom.twist.twist.linear.x = 0;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = 0;

		yaml_pose_pub_.publish(odom);

		ROS_ERROR("publish succeed");
		//geometry_msgs::PoseStamped test_msg;
		//poseStampedTFToMsg(tmp_pose,test_msg);
		//yaml_pose_pub_.publish(test_msg);
    }  
    catch(tf::TransformException e)
    {  
        ROS_ERROR("Failed to compute current pose, skipping labels (%s)", e.what());  
        return 1;  
    }
	//Label label2map_;
	double yaw,pitch,roll;  
  	tmp_pose.getBasis().getEulerYPR(yaw, pitch, roll);  
	label2map_.id = label_.id; 
	label2map_.x = tmp_pose.getOrigin().x();  
  	label2map_.y = tmp_pose.getOrigin().y(); 
	label2map_.z = tmp_pose.getOrigin().z(); 
	label2map_.th = yaw;
	ROS_ERROR("X:%f Y:%f th:%f",label2map_.x,label2map_.y,label2map_.th);
	return 0;
}

bool MapServer::findx(Label lab)
{
    //Label label2map_;
	return lab.id == label2map_.id;
	//return false;
}

void MapServer::createPoseFromLabel(const Label& l, tf::Pose& p)
{
  //t.setOrigin(tf::Vector3(l.x, l.y, l.z));
  //tf::Quaternion q;
  //q.setRPY(0.0, 0.0, l.th);
  //t.setRotation(q);
	tf::Quaternion q;
  	//q.setRPY(0, 0, -label_.th);
	//double x,y,z;
	//x = -(cos(-label_.th)*label_.x-sin(-label_.th)*label_.y);
	//y = -(sin(-label_.th)*label_.x+cos(-label_.th)*label_.y);
	//tf::Stamped<tf::Pose> label_pose(tf::Pose(q,tf::Vector3(-x, -y, -label_.z)),ros::Time(0), "usb_cam");
	q.setRPY(0, 0, l.th);
	p = tf::Pose(q,tf::Vector3(l.x, l.y, l.z));



}

bool MapServer::checkPose(const tf::Pose &pose_new)
{
	double dist_dt = 0.0;
	double angle_dt = 0.0;

	double ori = 0.0;
	double yaw,pitch,roll;
	pose_new.getBasis().getEulerYPR(yaw, pitch, roll);
	ori = tf::getYaw(cur_pose.pose.orientation);

	dist_dt = hypot((pose_new.getOrigin().x() - cur_pose.pose.position.x),(pose_new.getOrigin().y() -cur_pose.pose.position.y ));
	angle_dt = fabs(yaw - ori);
	ROS_INFO("new_pose:[%.3f, %.3f, %.3f], cur_pose[%.3f, %.3f, %.3f]",pose_new.getOrigin().x(),pose_new.getOrigin().y(),yaw,cur_pose.pose.position.x,cur_pose.pose.position.y, ori);
    ROS_INFO("dist and angle dist:%f,%f",dist_dt,angle_dt);
	if((dist_dt < distance_threshold_min)&&(angle_dt < angle_threshold_min))
	{	
        ROS_ERROR("The deta_angle:%.3f(<%.3f)",angle_dt,angle_threshold_min);	
		ROS_ERROR("The deta_dist:%.3f(<%.3f),don't set initial pose",dist_dt,distance_threshold_min);
		ros::param::get("start_pose_check",start_pose_check);
		if(!start_pose_check)
		{
			ros::param::set("start_pose_check",true);
		}
		return false;
	}

	/*if(angle_dt < angle_threshold_min)
	{
		ROS_ERROR("The deta_angle:%.3f(<%.3f)",angle_dt,angle_threshold_min);
		return false;
	}*/
	return true;
}

void MapServer::labelCallback(const ar_track_alvar_msgs::AlvarId::ConstPtr& msg)
{	
	Json::Value root;
	int msg_id;
	Json::FastWriter fast;
	std_msgs::String jsonStr;
	msg_id = msg->id;
	root["label_id"] = Json::Value(msg_id); 
	jsonStr.data = fast.write(root);
	app_label_pub_.publish(jsonStr);

	//check the navigation status
	if(!isNavState())
	{
		ROS_INFO("robot is not in navgation state now!");
		return;
	}

	if(!(tf_listener -> canTransform("base_link","map",ros::Time(0))))
	{
		ROS_INFO("No loaded maps");
		ros::Duration(0.2).sleep();
		return; 
	}
	
	ROS_DEBUG("AMCL NOW");
	//parse the camera information 
	if(parseLabelData(msg))
	{
		ROS_DEBUG("No labels are validly parsed!");
		//last_rcv_id_ = label_.id;
		return;
	}
  	//tf::TransformBroadcaster br;
  	//last_rcv_id_ = label_.id;
	//tf::Transform label_transform;
	ROS_INFO("Found Label! label_id_:%d;;;last_rcv_id_:%d",label_.id,last_rcv_id_);
	//ROS_INFO("label_id_:%d",label_.id);
	//last_rcv_id_ = label_.id;
	//ROS_INFO("last_rcv_id: %d;label_:%d",last_rcv_id_,label_.id);

  	std::vector<Label>::iterator iter;
	//boost::bind(&MapServer::findx,this,_1)
	//ROS_ERROR("vec 0 %d;1%d",label_vec_[0].id,label_vec_[1].id);
	for(iter = label_vec_.begin();iter != label_vec_.end();iter++)
	{	
		//ROS_INFO("%d",iter->id);
		if(iter->id == label_.id)
		{
			break;
		}	
	}
  	//iter = find_if(label_vec_.begin(), label_vec_.end(),findx);
  	if (iter != label_vec_.end())
	{
		//true
		ROS_INFO("Found the old label!");

		Label labelmap,camlabel;
		camlabel = label_;
		labelmap.x = iter->x;
		labelmap.y = iter->y;
		labelmap.z = iter->z;
		labelmap.th =iter->th;
		//last_rcv_id_ == label_.id;
		tf::Pose cam2labelpose,label2mappose;
		//createTfFromXYTheta()
		//label_: label->usb_cam
		//label2map: map->label
		//pose_new: map->base_linkb
		double x,y,z;
		createPoseFromLabel(camlabel,cam2labelpose);
		createPoseFromLabel(labelmap,label2mappose);
		x = label2mappose.getOrigin().x();  
  		y = label2mappose.getOrigin().y(); 
		z = label2mappose.getOrigin().z();
		//ROS_INFO("camlabel x:%f,y%f,z:%f",x,y,z);
		//ROS_INFO("yaml label::::x:%f,y%f,z:%f",labelmap.x,labelmap.y,labelmap.z);
		tf::StampedTransform bl2cam;
		//ROS_ERROR("1");
		//tf_listener -> lookupTransform("base_link","usb_cam_up",ros::Time(0),bl2cam);
		tf_listener -> lookupTransform("usb_cam_up","base_link",ros::Time(0),bl2cam);
		//tf_listener -> lookupTransform("base_link","map",ros::Time(0),bl2m);
		//ROS_ERROR("2");
		//bl2cam*cam2labelpose*bl2cam;
		//tf::Pose pose_new = bl2cam*label2mappose*cam2labelpose;
		
		tf::Pose camera_map_pose = label2mappose*cam2labelpose;
		ROS_INFO("camera_map_pose1: x:%f,y:%f",camera_map_pose.getOrigin().x(),camera_map_pose.getOrigin().y());
		//tf::Pose pose_new = bl2cam.inverse()*camera_map_pose;
		tf::Pose pose_new = camera_map_pose*bl2cam;
		bool set_pose_flag = checkPose(pose_new);//tf::pose check_pose = pose_new*inverseTimes(bl2m);s
		//tf::Pose pose_new = (cam2labelpose*label2mappose).inverse();
		//tf::Pose pose_new = (cam2labelpose*label2mappose).inverse()*bl2ca7m;
		if(!set_pose_flag)
		{	
			ROS_ERROR("Do not set pose this time");
			return;
		}
		double pose_yaw,pose_pitch,pose_roll;  
		pose_new.getBasis().getEulerYPR(pose_yaw, pose_pitch, pose_roll);
		//("th:%f,labe_th%f",labelmap.th,pose_yaw);
		geometry_msgs::PoseWithCovarianceStamped initialPose;
		initialPose.header.frame_id = "map";
		initialPose.header.stamp = ros::Time::now();// - ros::Duration(0.1);//triangle.laserm_A.stamp + ros::Duration(0.03);
		//initialPose.pose.pose = pose_new;
		initialPose.pose.pose.position.x = pose_new.getOrigin().x(); 
		initialPose.pose.pose.position.y = pose_new.getOrigin().y();
		initialPose.pose.pose.position.z = 0;

		tf::Quaternion q = tf::createQuaternionFromYaw(pose_yaw);
		geometry_msgs::Quaternion qMsg;
		tf::quaternionTFToMsg(q, qMsg);
		initialPose.pose.pose.orientation = qMsg;

		initialPose.pose.covariance[6*0+0] = label_initial_pose_covx;
		initialPose.pose.covariance[6*1+1] = label_initial_pose_covy;
		initialPose.pose.covariance[6*5+5] = label_initial_pose_covth;
		ROS_INFO("Setting pose: %.3f %.3f %.3f", pose_new.getOrigin().x(), pose_new.getOrigin().y(), pose_yaw);

		initial_pose_pub_.publish(initialPose);
		ros::param::get("start_pose_check",start_pose_check);
		if(!start_pose_check)
		{
			ros::param::set("start_pose_check",true);
		}
		geometry_msgs::PoseStamped cameraSetPoseRecord;
		cameraSetPoseRecord.header.stamp = ros::Time::now();
		cameraSetPoseRecord.header.frame_id = "map";
		cameraSetPoseRecord.pose.position.x = pose_new.getOrigin().x();
		cameraSetPoseRecord.pose.position.y = pose_new.getOrigin().y();
		cameraSetPoseRecord.pose.position.z = 0;
		cameraSetPoseRecord.pose.orientation = qMsg;
		camera_setpose_record_.publish(cameraSetPoseRecord);

		last_set_pose_time_ = ros::Time::now();
		/*
		nav_msgs::Odometry odom1;
		odom1.header.stamp = ros::Time::now();
		odom1.header.frame_id = "map";
		tf::poseTFToMsg(pose_new,odom1.pose.pose);
		//odom.child_frame_id = "label";
		odom1.twist.twist.linear.x = 0;
		odom1.twist.twist.linear.y = 0;
		odom1.twist.twist.angular.z = 0;

		//yaml_pose_pub_.publish(odom1);

		*/

		//setInitialPose();
		return;
	}
  	else
	{	ROS_INFO("Found the new label！");
		//false add label information to yaml
		
		if(auto_label_save_)
		{
			boost::get<2>(label_save_info_) = true;
		}
		if(boost::get<2>(label_save_info_) == true)
		{
			if(0!=label2Label2Map()) return;
			//ROS_ERROR("%s %s",(label_save_info_.get<0>()).c_str(),(label_save_info_.get<1>()).c_str());
			saveLabel2Yaml(label_save_info_.get<0>(),label_save_info_.get<1>());
			label_vec_.push_back(label2map_);
			boost::get<2>(label_save_info_) = false;
			last_rcv_id_ = label_.id;
			//ROS_INFO("Waiting for 3 seconds!");
			//ros::Duration(10.0).sleep();
			//if(save+label_ = true)
		}
		return;
	

	}





/*
	if(check_label_id_exist(label2map_.id))
	{
		//true
		ROS_INFO("Found the old label！");
		queryLabel();
		setInitialPose();
	}
	else
	{
		//false add label information to yaml
		ROS_INFO("Found the new label！");
    	if(0!=label2Label2Map(label_,label2map_))return ;	
		saveLabel2Yaml();
		label_vec_.push_back(label2map_);
	}
*/
	//return;	
	//tf::Stamped<tf::Pose> transformed_label2map;  
	//transform_listener = new tf::TransformListener();  
	//transform_listener.transformPose("map", label2bl, transformed_label2map); 

	/*
	geometry_msgs::PointStamped label_pose;
	label_pose.header.stamp=ros::Time();
	label_pose.header.frame_id="label_pose";
	label_pose.point.x=label_.x;
	label_pose.point.y=label_.y;
	label_pose.point.z=label_.z;
	*/
	//tf::Stamped<tf::Pose> tmp_pose;
	//tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), 
    //tf::Vector3(label_.x, label_.y, 0.0)), ros::Time(0), "label");
	//transformPose("map", ident, tmp_pose);
	//transformPoint("map",current_pose,tmp_pose,)

}
bool MapServer::parseLabelData(const ar_track_alvar_msgs::AlvarId::ConstPtr& msg)
{	

    std::deque<ar_track_alvar_msgs::AlvarId> label_msgs_cache_last = label_msgs_cache_;
    
    // 保存5个最近的msg作为缓存
    label_msgs_cache_.push_front(*msg);
    if(label_msgs_cache_.size() > 5)
    {
        label_msgs_cache_.resize(5);
    }
    else
    {
        return true;    // 缓存中数据不足五个的时候不作处理
    }

    id_to_num_.clear();
    // 统计缓存中id数目最多的那个id作为当前label的id
    for(auto iter = label_msgs_cache_.begin(); iter != label_msgs_cache_.end(); ++iter)
    {
        ++id_to_num_[(*iter).id];
    }

    label_.id = getMostIdFromMap(id_to_num_);
    last_rcv_id_ = label_.id;
    ROS_DEBUG("get most id: %d", label_.id);
    
    if(msg->id == -1 || label_.id == -1)   // 如果当前id为-1或者缓存中最大数目id是-1，都直接返回
	{
		ROS_DEBUG("label id -1, No labels currently!");
		return true;
	}

    // 统计上一个缓存中id数量最多th与当前th之差的平均值，如果平均差值过大，则舍去这一帧
    double th_diff_sum = 0;
    int th_num = 0;
    if(label_.id == last_rcv_id_)
    {
        for(auto iter = label_msgs_cache_last.begin(); iter != label_msgs_cache_last.end(); ++iter)
        {
            if((*iter).id == label_.id)
            {
                ++th_num;
                double diff = (*iter).th - msg->th;   // 计算当前角度与之前缓存中角度的偏差
                if(diff > M_PI)    // 将diff归一化到-pi到pi之间
                {
                    diff -= M_PI*2;
                }
                else if(diff < -M_PI)
                {
                    diff += M_PI*2;
                }
                th_diff_sum += diff;
            }
        }
        double th_diff_average = th_diff_sum/th_num;   // 取偏差平均值
        if(fabs(th_diff_average) > 0.9)   // 如果偏差平均值的绝对值大于0.9（约51度）
        {
            ROS_WARN("th_diff_sum: %f, th_num: %d, abnormal change of th, abandon this frame.", th_diff_sum, th_num);
            label_msgs_cache_.pop_front();   // 异常数据从缓存中除去
            return true;
        }

        if((ros::Time::now() - last_set_pose_time_) <= ros::Duration(reset_label_duration_double_))
		{
            ROS_INFO("need to wait for the reset duration, It is the same id as the last one!");
			return true;
		}

        label_.x = msg->x;
        label_.y = msg->y;
        label_.z = msg->z;
        label_.th = msg->th;
        ROS_DEBUG("parse label succeed");
        return false;
    }
    return true;
}


void MapServer::initCMDProcessor()
{
	cmd_processor_ = boost::shared_ptr<CMDProcessor>(new CMDProcessor());
}

void MapServer::initCMDThreadPool(int thread_num)
{
	thread_stop_ = false;
	threadpool_.create_thread(boost::bind(&MapServer::registerSystemThread, this))->detach();
	for(int i = 0; i < thread_num; i ++ )
		threadpool_.create_thread(boost::bind(&MapServer::taskThread,this))->detach();

}
void MapServer::taskThread()
{
	Task task;
	while(ros::ok()&&!thread_stop_)
	{
		{
			boost::mutex::scoped_lock lock(task_mtx_);

			//check in loop whether there are tasks to be processed
			while(cmd_processor_->empty()&&ros::ok()&&!thread_stop_)
			{
				//if no job, waiting here until notified
				cond_pull_.wait(task_mtx_);
			}
			//pop job
			this->cmd_processor_->popTask(task);

		}
		//process job
		task();
	}
}

//handle cmd and pub msg
void MapServer::taskRunner(const Func func,const Jptr jptr, std::string app_cmd,ros::Publisher & pub)
{
	std::string msg_info;int err;
	Json::Value root_out_;
	Json::Value dataArray;
	Json::FastWriter writer;
	//cmd function, special in .hpp file, dataArray, err and msg_info store the feedback msg
	func(jptr,dataArray,err,msg_info);

	//create json
	root_out_[SUB_NAME] = app_cmd;
	root_out_[DATA]= dataArray;
	root_out_[ERROR_CODE] = err;
	root_out_[MSG] = msg_info;
	std_msgs::String output_msg;
	output_msg.data = writer.write(root_out_);
	ROS_DEBUG("%s", writer.write(root_out_).c_str());
	pub.publish(output_msg);
	msg_info.clear();
}


inline void MapServer::registerSystem()
{
	node_state.enabled = 0;
	node_state.name = "Map Server";
	node_state.error_code = 0;
	node_state.info = "Robot Map Server Center";
	robot_state_keeper::RegisterState srv;
	srv.request.state = node_state;
	register_client_.call(srv);
}

/*
 *  register system state thread
 */
void MapServer::registerSystemThread()
{
	ros::Rate loop_rate(0.5);
	ROS_DEBUG("Map Server Register System Thread Start");
	while (ros::ok())
	{
		registerSystem();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

