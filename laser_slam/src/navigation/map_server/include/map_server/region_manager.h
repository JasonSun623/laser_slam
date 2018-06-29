/*
 * region_manager.h
 *
 *  Created on: Aug 30, 2017
 *      Author: tomzhang
 */

#ifndef INCLUDE_MAP_SERVER_REGION_MANAGER_H_
#define INCLUDE_MAP_SERVER_REGION_MANAGER_H_
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include "map_server/database_manager.h"
#include "motion_planner/MotionOp.h"
namespace map_server
{
class RegionManager
{
public:
	RegionManager(std::string root_path);
	~RegionManager();
	int getRegionByID(int region_id, Json::Value &data_in);
	int getRegionByID(int region_id, Region &region);
	int modifyRegion(Json::Value &data_in);
	int removeRegionByID(int region_id);
	int getRegionsByMap(Json::Value &data_in);
	int getRegionsByMap(std::vector<Region> &regions);
	int getRegionsByNav(int nav_type,Json::Value &data_in);
	int createRegion(Json::Value &data_in);
	int createRegion(Region &region);
	int openRegionsBD(std::string scene_name, std::string map_name);

	//void map_list_sub(const Json::Value & in);

	//void initRegionDB();
	//void jsonCallback(const std_msgs::String::ConstPtr data);
	void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
	void param_changer_fun();
	//bool handleCmd(const Json::Value& jdata, Json::Value& fb);
	//boost::mutex getUpdateMutex(){return update_mtx_;};
	//boost::mutex getDBMutex(){return db_loaded_mtx_;};
private:
	int createRegionID();
	void getMaxRegionID(int& region_id);
	void update();
	void checkAndSynDB();
	void checkTables();
	void addDefaultRegion();
	//void checkTableField(const std::string &table_name, const std::string &field_name)
	void getRegionsByType(int region_type, std::vector<Region> &regions);
	void initTables();
	void regionCheck(int region_type, Region &finded_region,const Region &default_region,const std::vector<Region>&regions,const geometry_msgs::Point &pos);
	bool robotPosCheck(const std::vector<Vertex> &vts, const geometry_msgs::Point &pos)const;
	void navParamsPub(Region &finded_region);
	void locParamsPub(const Region &finded_region);
	void audioParamsPub(const Region &finded_region);
	void sensorParamsPub(const Region &finded_region);
	bool params_service(motion_planner::MotionOp::Request  &req,
			motion_planner::MotionOp::Response &res);
	std::vector<Region> regions_;
	DataBaseManager *dbmgr_;
	ros::NodeHandle nh_,pnh_;
	ros::Publisher json_pub_,sensor_params_pub_,audio_params_pub_;
	ros::Subscriber json_sub_,map_list_sub_;
	boost::mutex update_mtx_,db_loaded_mtx_,odom_lock_;
	boost::condition db_loaded_cond_;
	ros::Subscriber odom_sub_;
	ros::ServiceClient fixpath_params_updater_;
	ros::ServiceServer params_service_;
	nav_msgs::Odometry base_odom_;
	boost::thread param_pub_thread_;

	std::string scene_name_,map_name_,path_;
	int param_check_rate_;
	bool region_updated_,db_loaded_;

	std::vector<Table> tables_;

};

}
#endif /* INCLUDE_MAP_SERVER_REGION_MANAGER_H_ */
