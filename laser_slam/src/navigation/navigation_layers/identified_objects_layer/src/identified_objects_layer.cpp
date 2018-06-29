/*
 * identified_objects_layer.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: tomzhang
 */
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include "costmap_2d/cost_values.h"
#include "identified_objects_layer/identified_objects_layer.h"
PLUGINLIB_EXPORT_CLASS(identified_objects_layer::IdentifiedObjectsLayer, costmap_2d::Layer)
using namespace identified_objects_layer;
IdentifiedObjectsLayer::IdentifiedObjectsLayer() {}
IdentifiedObjectsLayer::~IdentifiedObjectsLayer() {}
void IdentifiedObjectsLayer::onInitialize()
{
	ros::NodeHandle nh("~/" + name_);

	current_ = true;

	nh.param("enabled", enabled_, false);

	nh.param("keep_history_objects", keep_history_objects_, false);

	nh.param("keep_part", keep_part_, false);

	matchSize();

	min_x_ = min_y_ = std::numeric_limits<double>::max();

	max_x_ = max_y_ = std::numeric_limits<double>::min();

	this->setDefaultValue(costmap_2d::FREE_SPACE);

	global_frame_ = this->layered_costmap_->getGlobalFrameID();

	object_sub_ = nh.subscribe("/objects",100,&IdentifiedObjectsLayer::bufferIncomingObjectsMsgs,this);



}

void IdentifiedObjectsLayer::reset()
{
  ROS_DEBUG("Reseting Identified Objects Layer...");
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

void IdentifiedObjectsLayer::deactivate()
{
	objectsMsgs_.clear();
	objects_.clear();
	enabled_ = false;
}

void IdentifiedObjectsLayer::activate()
{
	enabled_ = true;
}
void IdentifiedObjectsLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y)
{

  	  if (layered_costmap_->isRolling())
   	  	updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

	  //ROS_ERROR("update");
	  if (!enabled_)
	  {
	    current_ = true;
	    return;
	  }
	  resetMaps();

	  updateCostmap();

	  *min_x = std::min(*min_x, min_x_);
	  *min_y = std::min(*min_y, min_y_);
	  *max_x = std::max(*max_x, max_x_);
	  *max_y = std::max(*max_y, max_y_);

	  min_x_ = min_y_ = std::numeric_limits<double>::max();
	  max_x_ = max_y_ = std::numeric_limits<double>::min();
 	  //ROS_ERROR("update finish");



}
void IdentifiedObjectsLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
	//ROS_ERROR("enable %d,update %d,",enabled_ ,updated_);
	if (!enabled_ ||!updated_)
	{
		ROS_ERROR("return");
		return;
	}
	//ROS_ERROR("update %d,%d,%d,%d",min_i, min_j, max_i, max_j);
	switch (0)
	{
	case 0:  // Overwrite
		updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
		break;
	case 1:  // Maximum
		updateWithMax(master_grid, min_i, min_j, max_i, max_j);
		break;
	default:  // Nothing
		break;
	}
	updated_ = false;
}
void IdentifiedObjectsLayer::bufferIncomingObjectsMsgs(const std_msgs::String::ConstPtr & msg)
{
	//ROS_INFO("msg in");
	boost::mutex::scoped_lock lock(objects_message_mutex_);
	objectsMsgs_.push_back(*msg);
}
void IdentifiedObjectsLayer::updateCostmap()
{
	processObjectsMsg();

	if(objects_.empty())
	{
		//ROS_ERROR("no objs");
		return;
	}

	updateCostmapUsingObjects();

	if(!updated_)
	{
		updated_ = true;
	}

}

void IdentifiedObjectsLayer::processObjectsMsg()
{
	std::list<std_msgs::String> msgs_copy;
	{
		boost::mutex::scoped_lock lock(objects_message_mutex_);
		msgs_copy = std::list<std_msgs::String>(objectsMsgs_);
		objectsMsgs_.clear();
	}
	if(msgs_copy.empty())
	{
		//ROS_ERROR("no msg");
		return;
	}
	std::vector<Object> objsl;
	for(std::list<std_msgs::String>::iterator it = msgs_copy.begin(); it != msgs_copy.end(); ++it)
	{
		createObjectsFromMsg(objsl,*it);
	}
	updateObjectsList(objsl);
}

bool IdentifiedObjectsLayer::createObjectsFromMsg(std::vector<Object> &objsl,const std_msgs::String &msg)const
{
	Json::Value objects;
	Json::Reader reader;
	if(reader.parse(msg.data,objects))
	{
		objsl.reserve(objects[OBJECTS].size());
		for(Json::ValueIterator it = objects[OBJECTS].begin(); it!=objects[OBJECTS].end(); ++it)
		{
				Object obj;
				if(createObjectFromJson(obj,*it))
				{
					objsl.push_back(obj);
				}
		}
		return true;
	}
	else
	{

		ROS_ERROR("Json Format Invalid, objects can not be parsed:%s",reader.getFormatedErrorMessages().c_str());
		return false;
	}
}
bool IdentifiedObjectsLayer::createObjectFromJson(Object &obj, const Json::Value &object) const
{

	ros::Duration duration(object[DURATION].asDouble());

	ros::Time stamp(object[STAMP].asDouble());


	if( ros::Time::now() - stamp > duration)
	{
		//return false;
	}
	std::string frame_id = object[FRAME_ID].asString();



	obj.duration = duration;
	obj.header.stamp = stamp;
	obj.header.frame_id = frame_id;

	ObjectsType type = (ObjectsType)(object[TYPE].asInt());

	switch(type)
	{

	case BED:
	case LITTLE_BED:
	{
		unsigned int footprint_size = object[FOOTPRINT].size();
		obj.object_footprint.resize(footprint_size);
		geometry_msgs::PointStamped pt;//_in,pt_out;
		for(unsigned int i = 0; i < footprint_size; ++i)
		{
			pt.header=obj.header;
			Json::Value pos = object[FOOTPRINT][i];
			pt.point.x = pos[(unsigned int)0].asDouble();
			pt.point.y = pos[1].asDouble();
			pt.point.z = pos[2].asDouble();
			//tf_->transformPoint (static_map_frame_, pt_in, pt_out);
			obj.object_footprint.push_back(pt);
		}
		break;
	}

	case LEG:
	{
		geometry_msgs::Point center;
		double radius = object[RADIUS].asDouble();
		center.x = object[CENTER][(unsigned int)0].asDouble();
		center.y = object[CENTER][1].asDouble();
		center.z = object[CENTER][2].asDouble();
		  geometry_msgs::PointStamped pt;
		  pt.header = obj.header;
		  for (int i = 0; i < 12; ++i)
		  {
		    double angle = i * 2 * M_PI / 12;
		    pt.point.x = center.x+cos(angle) * radius;
		    pt.point.y = center.y+sin(angle) * radius;
		    pt.point.z = 0.0;
		    obj.object_footprint.push_back(pt);
		    //ROS_ERROR("circle %f,%f,radius:%f",pt.point.x,pt.point.y,radius);
		  }
		
		break;
	}

	}





	/*
	 // we transform object from base_link to map
	if(!tf_->waitForTransform(obj.header.frame_id, frame_id,
			obj.header.stamp, ros::Duration(0.5)) )
	{
		ROS_ERROR_THROTTLE(1.0, "Identified Objects Layer can't transform from %s to %s at %f",
				obj.header.frame_id.c_str(), frame_id.c_str(),
				obj.header.stamp.toSec());
		return false;
	}

	*/



	return true;
}

void IdentifiedObjectsLayer::updateObjectsList(const std::vector<Object> &objsl)
{
	std::list<Object>::iterator it = objects_.begin();
	if(keep_history_objects_)
	{
		for(;it!=objects_.end();)
		{
			if(ros::Time::now() - it->header.stamp < it->duration )
			{
				++it;
			}
			else
			{
				objects_.erase(it++);
			}
		}
	}
	else
	{
		objects_.clear();
	}
	std::copy(objsl.begin(),objsl.end(),std::back_inserter(objects_));
	//ROS_ERROR("obj size: %d",int(objects_.size()));
}
void IdentifiedObjectsLayer::updateCostmapUsingObjects()
{
	std::list<Object>::iterator it = objects_.begin();
	for(;it!=objects_.end();)
	{

		if(setPolygonOutlineCost(it->object_footprint,costmap_2d::LETHAL_OBSTACLE))
		{
			++it;
			continue;
		}
		objects_.erase(it++);
	}
}
bool IdentifiedObjectsLayer::setPolygonOutlineCost(const std::vector<geometry_msgs::PointStamped>& polygon, unsigned char cost_value)
{

	// we transform polygon from map frame to odom
	if(!tf_->waitForTransform(global_frame_, polygon.front().header.frame_id,
			polygon.front().header.stamp, ros::Duration(0.1)) )
	{
			ROS_ERROR_THROTTLE(1.0, "Identified Objects Layer can't transform from %s to %s at %f",
					global_frame_.c_str(),polygon.front().header.frame_id.c_str(),
					polygon.front().header.stamp.toSec());
			return false;
	}

	double min_x,min_y,max_x,max_y;
	min_x = min_y = std::numeric_limits<double>::max();
	max_x = max_y = std::numeric_limits<double>::min();
	// we assume the polygon is given in the global_frame... we need to transform it to map coordinates

	std::vector<costmap_2d::MapLocation> map_polygon;
	geometry_msgs::PointStamped pt;
	for (unsigned int i = 0; i < polygon.size(); ++i)
	{
    		try
    		{
			tf_->transformPoint(global_frame_,polygon[i],pt);
			costmap_2d::MapLocation loc;
			if (!worldToMap(pt.point.x, pt.point.y, loc.x, loc.y))
			{
				if(!keep_part_)
				{
				// ("Polygon lies outside map bounds, so we can't fill it");
					return false;
				}
				else
				{
					continue;
				}
			}
			//ROS_ERROR("polygon %f,%f",polygon[i].point.x,polygon[i].point.y);
			touch(pt.point.x, pt.point.y,&min_x,&min_y,&max_x,&max_y);
			map_polygon.push_back(loc);
    		}
    		catch (tf::TransformException& ex)
    		{	
      			ROS_ERROR("TF Error attempting to transform an polygon from %s to %s: %s", global_frame_.c_str(),
                	polygon.front().header.frame_id.c_str(), ex.what());
      			return false;
    		}


	}
	if(map_polygon.empty())
	{
		return false;
	}
	touch(min_x,min_y,&min_x_,&min_y_,&max_x_,&max_y_);
	touch(max_x,max_y,&min_x_,&min_y_,&max_x_,&max_y_);
	//ROS_ERROR("x y %f,%f %f,%f,%f,%f %f,%f",min_x,min_y,max_x,max_y,min_x_,min_y_,max_x_,max_y_);
	std::vector<costmap_2d::MapLocation> polygon_cells;

	// get the cells that make up the outline of the polygon
	polygonOutlineCells(map_polygon, polygon_cells);

	// set the cost of those cells
	for (unsigned int i = 0; i < polygon_cells.size(); ++i)
	{
		unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
		costmap_[index] = cost_value;
		//ROS_ERROR("set %d,%d, cost %u",polygon_cells[i].x, polygon_cells[i].y,cost_value);
	}
	return true;
}
