/*
 * map_server_mrobot.h
 *
 *  Created on: Apr 6, 2017
 *      Author: tomzhang, Xumi, Gavin, Paul
 */

#ifndef MAP_SERVER_INCLUDE_MAP_SERVER_MAP_SERVER_MROBOT_H_
#define MAP_SERVER_INCLUDE_MAP_SERVER_MAP_SERVER_MROBOT_H_

#include "yaml-cpp/yaml.h"
#include "map_server/image_loader.h"
#include "map_server/map_list_manager.h"
#include "map_server/cmd_processor.h"
#include "map_server/json_string_variables.h"
#include "map_server/query_uwb.h"
#include "map_server/GetFixPath.h"
#include "map_server/query_all_points.h"
#include "robot_state_keeper/robot_state.h"
#include "robot_state_keeper/RegisterState.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h" 
#include "tf/transform_listener.h"  
#include "tf/message_filter.h" 
#include "tf/transform_datatypes.h"
#include "tf/tf.h"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <stdexcept>
#include <map>
#include <sys/stat.h>

#include "json/json.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/atomic.hpp>
#include <ar_track_alvar_msgs/AlvarId.h>
#include <boost/tuple/tuple.hpp>

#include "map_server/region_manager.h"
#include "gmapping/map_filter.h"

const static int UWB_POINT_TYPE = 3;

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
	i = node.as<T>();
}

#endif

namespace map_server {

typedef struct{
    unsigned int id;
    unsigned int type;
    double length;
    std::string scene_name;
    std::string map_name;
    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped end;
    std::vector<geometry_msgs::PoseStamped> points;
    std::vector<geometry_msgs::PoseStamped> path;
}path_t;

typedef struct{
    unsigned int id;
    unsigned int type;
    std::string scene_name;
    std::string map_name;
    geometry_msgs::Pose2D pose;
}point_t;


void int2str(const int &int_temp, std::string &string_temp);
// point struct, store the information of point.
struct Point
{


	double x;
	double y;
	double th;
	int type;
	std::string nm;
	std::string alias;
	std::string map_name;
};

struct Label
{
	double x;
	double y;
	double z;
	double th;
	int id;
	void reset();
};


class Shape
{
public:
	int id;
	std::string name;
	std::string tag;
	Shape():name("Shape"){};
	virtual ~Shape(){};
	virtual void getInterpolationPoints(std::vector<Point>  &points, double step ){};
	virtual void convertToJson(Json::Value &dataArray){};
	virtual void convertFromJson(const Json::Value &dataArray){};
	virtual void convertToYAML(YAML::Node &doc){};
	virtual void convertFromYAML(const YAML::Node::const_iterator &doc){};
	virtual bool deleteInYAML(YAML::Node &doc){return 0;};
};

class Line : public Shape
{
public:
	Line(){name=("Line");};
	~Line(){};
	double start_x;
	double start_y;
	double end_x;
	double end_y;
	void getInterpolationPoints(std::vector<Point>  &points, double step );
	void convertToJson(Json::Value &dataArray);
	void convertFromJson(const Json::Value &dataArray);
	void convertToYAML(YAML::Node &doc);
	void convertFromYAML(const YAML::Node::const_iterator &doc);
	bool deleteInYAML(YAML::Node &doc);
};


class Polygon : public Shape
{
public:
	Polygon(){name=("Polygon");};
	~Polygon(){};
	std::vector<Line> linesegments;
	void getInterpolationPoints(std::vector<Point>  &points, double step );;
	void convertToJson(Json::Value &dataArray);
	void convertFromJson(const Json::Value &dataArray);
	void convertToYAML(YAML::Node &doc);
	void convertFromYAML(const YAML::Node::const_iterator &doc);
	bool deleteInYAML(YAML::Node &doc);
};
class Circle : public Shape
{
public:
	Circle(){name=("Circle");};
	~Circle(){};
	double radius;
	double x;
	double y;
	void getInterpolationPoints(std::vector<Point>  &points, double step );
	void convertToJson(Json::Value &dataArray);
	void convertFromJson(const Json::Value &dataArray);
	void convertToYAML(YAML::Node &doc);
	void convertFromYAML(const YAML::Node::const_iterator &doc);
	bool deleteInYAML(YAML::Node &doc);
};

class MapServer
{
public:
	/** Trivial constructor */
	explicit MapServer(std::string path, double res);
	~MapServer();
	void jsonCallback(const std_msgs::String::ConstPtr msg);
	void jsonRegionCallback(const std_msgs::String::ConstPtr msg);
	void mapSaverCallback(const nav_msgs::OccupancyGridConstPtr map);


	int label2Label2Map();
	int saveLabel2Yaml(const std::string &scene_name,const std::string &map_nam);
	int labelSaveExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str,std::string &msg_info);
	int labelDeleteDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info);
	void updateLabelVector();
	void queryLabel();
	//void setInitialPose();
	int addLabelYAML(const std::string fname);
	bool check_label_id_exist(const int id);
	bool parseLabelData(const ar_track_alvar_msgs::AlvarId::ConstPtr& msg);
	bool checkPose(const tf::Pose &pose_new);
	void labelCallback(const ar_track_alvar_msgs::AlvarId::ConstPtr& msg);
    int getMostIdFromMap(std::map<int, int> &id_to_num);

	void mapEditCallback(const nav_msgs::OccupancyGridConstPtr &map);
	void initFuncsMap();
	void initCMDProcessor();
	void initCMDThreadPool(int thread_num = 1);
	void save_map_thread(const std::string &save_scene_name, const std::string &save_fname);
	int  folder_check(const std::string& save_scene_name, std::string &msg_info);
	int  save_map_to_file(const nav_msgs::OccupancyGridConstPtr map, const std::string &save_scene_name, const std::string &save_fname, std::string &msg_info);
	void label_save(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void label_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void map_list_get(Jptr jptr,  Json::Value &dataArray, int &err, std::string & msg_info);
	void map_current_get(Jptr jptr,  Json::Value &dataArray, int &err, std::string & msg_info);
	void map_rename(Jptr jptr,  Json::Value &dataArray, int &err, std::string & msg_info);
	void map_scene_rename(Jptr jptr,  Json::Value &dataArray, int &err, std::string & msg_info);
	void map_delete(Jptr jptr,  Json::Value &dataArray, int &err, std::string & msg_info);
	void map_scene_delete(Jptr jptr,  Json::Value &dataArray, int &err, std::string & msg_info);
	void map_save(Jptr jptr,  Json::Value &dataArray, int &err, std::string & msg_info);
	void map_load(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void map_edit(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void map_edit_complete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void map_default_set(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void map_default_get(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);

	// point func
	void point_add(Jptr jptr,  Json::Value &dataArray, int &err, std::string & msg_info);
	void point_load(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void point_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void point_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void point_edit(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	// virtual wall func
	void virtual_wall_load(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void virtual_wall_add(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void virtual_wall_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);

	int getId(std::vector<int> id_vec);

	// regions func
	void regions_scene_get(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void regions_map_get(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void region_modify(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void region_create(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void region_remove(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void region_params_get(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);

	void fixpath_param_add(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void fixpath_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
    void fixpath_map_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
    void fixpath_path_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void fixpath_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void fixpath_edit(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void fixpath_record_start(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void fixpath_record_finish(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	void fixpath_file_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);


	//void label_add(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);
	//void label_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info);

	int labelVectorInit(const std::string &scene_name,const std::string &map_name);
	int loadLabelYAML(const std::string fname);
	std::vector<Label> label_vec_;
	int last_rcv_id_;
	Label label_;
	Label label2map_;
	//ros::Duration reset_label_duration_;
	ros::Time last_set_pose_time_;
	double reset_label_duration_double_;
	double distance_threshold_max;
	double angle_threshold_max;
	double distance_threshold_min;
	double angle_threshold_min;
	double label_initial_pose_covx;
	double label_initial_pose_covy;
	double label_initial_pose_covth;

	//<scene_name,map_name,save_flag>
	boost::tuple<std::string,std::string,bool> label_save_info_;
	bool findx(Label lab);
	void createPoseFromLabel(const Label& l, tf::Pose& p); 
	void current_pose_callback(const nav_msgs::Odometry& msg);


	void taskThread();
	void taskRunner(const Func func,const Jptr jptr, std::string app_cmd,ros::Publisher & pub);
	void registerSystemThread();
	void registerSystem();

private:

	int checkString(const std::string str, std::string &msg_info);

	inline bool isMapLoad();
	inline void setMapLoad(bool load);
	void loadAllMaps(const std::string& scene);
	int renameMap(const std::string oldname,
			const std::string newname, const std::string scene, std::string & msg_info);
	int checkMapName(const std::string &scene, const std::string &map);
	int readPGM(const std::string& fname, boost::shared_ptr<nav_msgs::GetMap::Response> &rep_ptr);
	int readMapYAML(std::string &mapfname, const std::string &fname, double &res,
			int &negate, double &occ_th, double &free_th, double* origin,
			MapMode &mode);
	int loadPointYAML(const std::string fname, std::vector<Point> &vec, std::string &msg_info);
	int writePointYAML(const std::string fname, Point &p, std::string &msg_info);
	int deletePointYAML(const std::string fname, const Point &p, std::string &msg_info);
	int editPointYAML(const std::string fname, const std::string point_name_str, const Point &p, std::string &msg_info);
	int queryPointYAML(const std::string fname, Point &p, std::string &msg_info);
	int loadVirtualWallYAML(const std::string fname, std::vector<boost::shared_ptr<Shape> > &vec, std::string &msg_info);
	int writeVirtualWallYAML(const std::string fname, std::vector<boost::shared_ptr<Shape> > &vec, std::string &msg_info, std::vector<int> shapes_id);
	int deleteVirtualWallYAML(const std::string fname, std::vector<boost::shared_ptr<Shape> > &vec, std::string &msg_info);
	int writeLabelYAML(const std::string fname, const Label l, std::string &msg_info);
	int deleteLabelYAML(const std::string fname, const Label l, std::string &msg_info);

	int mapRenameDataExtract(Jptr jptr, std::string &scene_name,std::string &old_name_str,std::string &new_name_str, std::string &msg_info);
	int mapSceneRenameDataExtract(Jptr jptr,std::string &old_scene_str, std::string &new_scene_str, std::string &msg_info);
	int mapDeleteDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info);
	int mapSceneDeleteDataExtract(Jptr jptr,std::string &scene_name_str, std::string &msg_info);
	int mapSaveDataExtract(Jptr jptr,std::string &save_scene_name, std::string &save_fname, std::string &msg_info);
	int mapLoadDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info);
	int pointLoadDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info);
	int pointAddDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str,Point &p, std::string &msg_info);
	int pointDeleteExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str,Point &p, std::string &msg_info);
	int pointQueryDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str,Point &p, std::string &msg_info);
	int pointEditDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str,std::string &point_name_str, Point &p,std::string &msg_info);
	int virtualWallDeleteExtract(Jptr jptr,std::string &scene_name_str,std::string &map_name_str, std::vector<boost::shared_ptr<Shape> > &vec, std::string &msg_info);
	int virtualWallLoadDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info);
	int virtualWallAddDataExtract(Jptr jptr,std::string &scene_name_str,std::string &map_name_str, std::vector<boost::shared_ptr<Shape > > &vec,std::string &msg_info);
	int labelAddDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info);	
	int createVirtualWallPointCloud(std::vector<boost::shared_ptr<Shape> > &in, sensor_msgs::PointCloud2 &cloud_out, const double &step, std::string &msg_info);

	int createJson();
	bool isNavState();
	bool queryUwbPoints(map_server::query_uwb::Request &req, map_server::query_uwb::Response &res);
	bool queryAllPoints(map_server::query_all_points::Request &req, map_server::query_all_points::Response &res);

	//fix path func
	void publish_fixpath(const std::vector<geometry_msgs::PoseStamped>& path);
	int  find_unuse_path_id(std::string scene_name, std::string map_name);
	bool get_line_start_end(const Json::Value &data,path_t &p);
	bool create_line_path(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal,std::vector<geometry_msgs::PoseStamped>& global_plan);
    bool get_bezier_curve_point(const Json::Value &data,path_t &p);
    bool create_bezier_curve_path(std::vector<geometry_msgs::PoseStamped> &path);
	bool save_path_database(path_t path);
	bool get_fixpath_info_by_map(std::string scene_name,std::string map_name,std::vector<path_t> &paths);
	bool get_fixpath_info_by_scene(const std::vector<std::string> &map_list,std::string scene_name,Json::Value &path,const std::vector<point_t> &points, bool query_all_points = false);
    bool get_fixpath_info_by_id(int id, std::string map_name,std::string scene_name,Json::Value &path,const std::vector<point_t> &points);
	bool get_all_points(const std::vector<std::string> &map_list,std::string scene_name,std::vector<point_t> &points);
	bool check_same_point(const geometry_msgs::Pose2D p1,const geometry_msgs::PoseStamped p2);
	bool delete_fixpath(path_t path);
	double line_dist(double x0,double y0,double x1,double y1);
	double adjust_th(double th);
    double calculate_path_length(std::vector<geometry_msgs::PoseStamped>& path);
	bool get_fixpath_service(map_server::GetFixPath::Request &req,map_server::GetFixPath::Response &res);
	bool write_string_to_file(std::string scene_name,const std::string & text, std::string file_name);
	void pub_map_filter(const nav_msgs::OccupancyGrid map);	


	ros::NodeHandle n_;
	ros::NodeHandle private_nh_;
	ros::Publisher map_pub_;
	ros::Publisher map_filter_pub_;
	ros::Publisher metadata_pub;
	ros::Publisher app_pub_;
	ros::Publisher app_param_pub_;
	ros::Publisher map_list_pub_;
	ros::Publisher app_label_pub_;
	ros::Subscriber app_sub_;
	ros::Subscriber map_sub_;
	ros::Subscriber app_map_sub_;
	ros::Subscriber app_param_sub_;
	ros::Subscriber current_pose_sub;
	ros::Publisher virtual_wall_pub;
	ros::Publisher estimate_current_pose_pub;
	ros::Publisher initial_pose_pub_;
	ros::Subscriber label_sub_;
	ros::Publisher yaml_pose_pub_;
	ros::Publisher camera_setpose_record_;
	ros::ServiceClient status_client_;
	//ros::Subscriber current_pose_;
	tf::TransformListener* tf_listener;
	bool auto_label_localization_;
	bool auto_label_save_;
	ros::ServiceClient convert_client_; 
	ros::ServiceServer query_uwb_service;
	ros::ServiceServer query_all_points_service;

	double step_;
	double res_;
	bool deprecated;
	//boost::atomic<bool> start_save_map_;
	boost::atomic<bool> start_edit_map_;
	boost::atomic<bool> thread_stop_;
	bool map_loaded;

	boost::mutex task_mtx_;
	boost::condition cond_pull_;
	boost::mutex save_map_mtx_;
	boost::mutex edit_map_mtx_;
	//boost::condition save_map_cond_;
	boost::condition edit_map_cond_;
	boost::thread_group threadpool_;
	nav_msgs::MapMetaData meta_data_message_;
	boost::shared_ptr<nav_msgs::GetMap::Response> map_resp_;
	nav_msgs::OccupancyGridConstPtr save_map_ptr_;
	nav_msgs::OccupancyGridConstPtr edit_map_ptr_;
	const std::string config_map_path_;
	std::string current_scene_;
	std::string current_map_;
	std::map<std::string, boost::shared_ptr<nav_msgs::GetMap::Response> > maps_;
	FuncMapPtr function_map_ptr_;
	FuncMapPtr function_region_ptr_;
	boost::shared_ptr<CMDProcessor> cmd_processor_;
	boost::shared_ptr<MapListManager> map_list_manager_;
	RegionManager *rmgr_;
	robot_state_keeper::robot_state node_state;
	// call system keeper state client
	ros::ServiceClient register_client_;
	geometry_msgs::PoseStamped cur_pose;


	ros::Publisher fixpath_plan_pub_;
	ros::ServiceServer get_fixpath_srv;
	ros::ServiceClient query_all_points_client;
	nav_msgs::Odometry fixpath_current_pose_;
	std::vector<geometry_msgs::PoseStamped> record_path;
	std::vector<path_t> all_path;
	bool record_path_start;
	bool start_pose_check;

	gmapping::map_filter map_filter_;

    std::map<int, int> id_to_num_;
    std::deque<ar_track_alvar_msgs::AlvarId> label_msgs_cache_;
};

}


#endif /* MAP_SERVER_INCLUDE_MAP_SERVER_MAP_SERVER_MROBOT_H_ */
