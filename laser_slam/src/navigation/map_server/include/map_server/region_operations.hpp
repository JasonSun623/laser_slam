/*
 * region_operations.hpp
 *
 *  Created on: Oct 24, 2017
 *      Author: Tom Zhang
 */

#ifndef MAP_SERVER_INCLUDE_REGION_OPERATIONS_HPP_
#define MAP_SERVER_INCLUDE_REGION_OPERATIONS_HPP_
#include "map_server/map_server_mrobot.h"
using namespace map_server;
/*
bool checkJson(const Json::Value& jdata, const std::string& tag, Json::ValueType type)
{
	if(jdata[tag].isNull())
	{
		ROS_ERROR("json tag %s cant find!", tag.c_str());
		return false;
	}

	switch (type)
	{
	case Json::intValue:
	case Json::booleanValue:
	case Json::realValue:
	{
		if(!jdata[tag].isNumeric())
		{
			ROS_ERROR("json tag %s value type is NaN!", tag.c_str());
			return false;
		}
		break;
	};
	case Json::stringValue:
	{
		if(!jdata[tag].isString())
		{
			ROS_ERROR("json tag %s value type is NaS!", tag.c_str());
			return false;
		}
		break;
	};
	case Json::objectValue:
	{
		if(!jdata[tag].isObject())
		{
			ROS_ERROR("json tag %s value type is NaO!", tag.c_str());
			return false;
		}
		break;
	};
	case Json::arrayValue:
	{
		if(!jdata[tag].isArray())
		{
			ROS_ERROR("json tag %s value type is NaA!", tag.c_str());
			return false;
		}
		break;
	};

	default:;

	};
	return true;

};
*/
void MapServer::regions_scene_get(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
//	if(!checkJson((*jptr),DATA,Json::objectValue))
//	{
//		msg_info = "json invalid!";
//		err = 2;
//		return;
//	}
//	Json::Value data_in = (*jptr)[DATA];
//	if(!checkJson(data_in,SCENE_NAME,Json::stringValue))
//	{
//		msg_info = "json invalid!";
//		err = 2;
//		return;
//	}
//	std::string scene_name = data_in[SCENE_NAME].asString();
//	if(scene_name != this->current_scene_)
//	{
//		err = 4;
//		msg_info = "Please load scene " +scene_name + " first!";
//		return;
//	}
//
//	Region region;
//	if(0!=(err =this->rmgr_->getRegionByID(0,region)))
//	{
//		msg_info = "database error!";
//		return;
//	}
//	std::vector<Region> regions;
//	err =this->rmgr_->getRegionsByScene(scene_name,regions);
//	if(1==err)
//	{
//		msg_info = "database error!";
//		return;
//	}
//	err = 0;
//	regions.push_back(region);
//	Json::Value data_out;
//	for(int i = 0 ; i<regions.size();++i)
//	{
//		Json::Value reg;
//		regions[i].convertToJson(reg);
//		data_out[REGIONS].append(reg);
//	}
//	dataArray[DATA] = data_out;
}
void MapServer::regions_map_get(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	if(!checkJson((*jptr),DATA,Json::objectValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	Json::Value data_in = (*jptr)[DATA];
	if(!checkJson(data_in,SCENE_NAME,Json::stringValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	if(!checkJson(data_in,MAP_NAME,Json::stringValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	std::string scene_name = data_in[SCENE_NAME].asString();
	std::string map_name = data_in[MAP_NAME].asString();
	if(scene_name != this->current_scene_)
	{
		err = 4;
		msg_info = "Please load scene " + scene_name + " first!";
		return;
	}
	if(map_name != this->current_map_)
	{
		err = 4;
		msg_info = "Please load map " + map_name + " first!";
		return;
	}

	Json::Value data_out;

	err =this->rmgr_->getRegionsByMap(data_out);
	if(1==err)
	{
		msg_info = "database error!";
		return;
	}
	err = 0;



	dataArray = data_out;
	return;
}
void MapServer::region_create(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	if(!checkJson((*jptr),DATA,Json::objectValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	Json::Value data_in = (*jptr)[DATA];
	std::string scene_name = data_in[SCENE_NAME].asString();
	std::string map_name = data_in[MAP_NAME].asString();
	if(scene_name != this->current_scene_)
	{
		err = 4;
		msg_info = "Please load scene " +scene_name + " first!";
		return;
	}
	if(map_name != this->current_map_)
	{
		err = 4;
		msg_info = "Please load map " + map_name + " first!";
		return;
	}

	if(0!=(err =this->rmgr_->createRegion(data_in)))
	{
		msg_info = "database error!";
		return;
	}
	dataArray = data_in;
	return;
}
void MapServer::region_remove(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	if(!checkJson((*jptr),DATA,Json::objectValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	Json::Value data_in = (*jptr)[DATA];
	dataArray[DATA] = data_in;
	if(!checkJson(data_in,REGION_ID,Json::intValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	if(!checkJson(data_in,SCENE_NAME,Json::stringValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	if(!checkJson(data_in,MAP_NAME,Json::stringValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	int id = data_in[REGION_ID].asInt();
	if(id == 0)
	{
		err = 4;
		return;
	}
	std::string scene_name = data_in[SCENE_NAME].asString();
	std::string map_name = data_in[MAP_NAME].asString();
	if(scene_name != this->current_scene_)
	{
		err = 4;
		msg_info = "Please load scene " +scene_name + " first!";
		return;
	}
	if(map_name != this->current_map_)
	{
		err = 4;
		msg_info = "Please load map " + map_name + " first!";
		return;
	}
	if(0!=(err =this->rmgr_->removeRegionByID(id)))
	{
		msg_info = "database error!";
		return;
	}
	return;
}
void MapServer::region_params_get(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	if(!checkJson((*jptr),DATA,Json::objectValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	Json::Value data_in = (*jptr)[DATA];
	if(!checkJson(data_in,REGION_ID,Json::intValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	if(!checkJson(data_in,SCENE_NAME,Json::stringValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	if(!checkJson(data_in,MAP_NAME,Json::stringValue))
	{
		msg_info = "json invalid!";
		err = 2;
		return;
	}
	int id = data_in[REGION_ID].asInt();
	std::string scene_name = data_in[SCENE_NAME].asString();
	std::string map_name = data_in[MAP_NAME].asString();
	if(scene_name != this->current_scene_)
	{
		err = 4;
		msg_info = "Please load scene " +scene_name + " first!";
		return;
	}
	if(map_name != this->current_map_)
	{
		err = 4;
		msg_info = "Please load map " + map_name + " first!";
		return;
	}
	Json::Value region;
	if(0!=(err =this->rmgr_->getRegionByID(id,region)))
	{
		msg_info = "database error!";
		return;
	}

	dataArray = region;
    dataArray[MAP_NAME] = map_name;
	dataArray[SCENE_NAME] = scene_name;
	return;
}
void MapServer::region_modify(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	if(!checkJson((*jptr),DATA,Json::objectValue))
	{
		err = 2;
		msg_info = "json invalid!";
		return;
	}
	Json::Value data_in = (*jptr)[DATA];
	std::string scene_name = data_in[SCENE_NAME].asString();
	std::string map_name = data_in[MAP_NAME].asString();
	if(scene_name != this->current_scene_)
	{
		err = 4;
		msg_info = "Please load scene " +scene_name + " first!";
		return;
	}
	if(map_name != this->current_map_)
	{
		err = 4;
		msg_info = "Please load map " + map_name + " first!";
		return;
	}
	err = this->rmgr_->modifyRegion(data_in);
	if(err == 1)
	{
		msg_info = "database error!";
		return;
	}
	else if(err ==2)
	{

	}
	dataArray = data_in;
	return;
}



#endif /* MAP_SERVER_INCLUDE_REGION_OPERATIONS_HPP_ */
