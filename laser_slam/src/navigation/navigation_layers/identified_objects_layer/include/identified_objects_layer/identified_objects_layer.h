/*
 * identified_objects_layer.h
 *
 *  Created on: Jan 3, 2018
 *      Author: tomzhang
 */

#ifndef NAVIGATION_LAYERS_IDENTIFIED_OBJECTS_LAYER_INCLUDE_IDENTIFIED_OBJECTS_LAYER_IDENTIFIED_OBJECTS_LAYER_H_
#define NAVIGATION_LAYERS_IDENTIFIED_OBJECTS_LAYER_INCLUDE_IDENTIFIED_OBJECTS_LAYER_IDENTIFIED_OBJECTS_LAYER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include "json/json.h"
static const std::string OBJECTS = "objects";
static const std::string FRAME_ID = "frame_id";
static const std::string STAMP = "stamp";
static const std::string TYPE = "type";
static const std::string FOOTPRINT = "footprint";
static const std::string DURATION = "duration";
static const std::string RADIUS = "radius";
static const std::string CENTER = "center";

namespace identified_objects_layer
{

struct Object
{
	std::vector<geometry_msgs::PointStamped> object_footprint;
	std_msgs::Header header;
	ros::Duration duration;
};

enum ObjectsType
{
	LEG,
	BED,
	LITTLE_BED
};
class IdentifiedObjectsLayer:public costmap_2d::CostmapLayer
{
public:
	/*
	 * constructor
	 */
	IdentifiedObjectsLayer();
	/*
	 * deconstructor
	 */
	~IdentifiedObjectsLayer();


	virtual void onInitialize();

	virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
			double* max_y);

	virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

	virtual void reset();

	virtual void deactivate();

	virtual void activate();
private:

	void bufferIncomingObjectsMsgs(const std_msgs::String::ConstPtr & msg);

	void processObjectsMsg();

	void processObjectsMsg(const std_msgs::String &msg);

	bool createObjectsFromMsg(std::vector<identified_objects_layer::Object> &objsl,const std_msgs::String &msg)const;

	bool createObjectFromJson(identified_objects_layer::Object &obj, const Json::Value &object) const;

	void updateObjectsList(const std::vector<identified_objects_layer::Object> &objsl);

	void updateCostmap();

	void updateCostmapUsingObjects();

	bool setPolygonOutlineCost(const std::vector<geometry_msgs::PointStamped>& polygon, unsigned char cost_value);


	boost::mutex objects_message_mutex_;

	double min_x_ ,min_y_ ,max_x_ ,max_y_ ;

	bool updated_,keep_history_objects_,keep_part_;

	std::string global_frame_,static_map_frame_;

	std::list<std_msgs::String> objectsMsgs_;

	std::list<identified_objects_layer::Object> objects_;

	ros::Subscriber object_sub_;
};

}
#endif /* NAVIGATION_LAYERS_IDENTIFIED_OBJECTS_LAYER_INCLUDE_IDENTIFIED_OBJECTS_LAYER_IDENTIFIED_OBJECTS_LAYER_H_ */
