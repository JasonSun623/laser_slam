/*
 * param.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: tomzhang
 */
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include "map_server/region.h"
#include "map_server/json_string_variables.h"


using namespace map_server;
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


Region::Region()
{

}
Region::~Region()
{

}
void Region::convertToJson(Json::Value& jdata)
{
	//jdata[SCENE_NAME] = scene_name;
	//jdata[MAP_NAME] = map_name;
	jdata[REGION_NAME] = region_name;
	jdata[REGION_ID] = region_id;
    jdata[REGION_TYPE] =region_type;
	Json::Value vtxs,pars;
	for(std::vector<Vertex>::iterator it = vts.begin();it!=vts.end();++it)
	{
		Json::Value vtxs;
		it->convertToJson(vtxs);
		jdata[VERTEX].append(vtxs);

	}
	param.convertToJson(pars);
	jdata[PARAMS] = pars;
}

int Region::convertFromJson(const Json::Value& jdata)
{
//	if(!checkJson(jdata,SCENE_NAME,Json::stringValue))
//	{
//		return 2;
//	}
//	if(!checkJson(jdata,MAP_NAME,Json::stringValue))
//	{
//		return 2;
//	}
	if(!checkJson(jdata,REGION_NAME,Json::stringValue))
	{
		return 2;
	}
	if(!checkJson(jdata,REGION_ID,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,REGION_TYPE,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,VERTEX,Json::arrayValue))
	{
		return 2;
	}
	if(!checkJson(jdata,PARAMS,Json::objectValue))
	{
		return 2;
	}
	this->vts.resize(jdata[VERTEX].size());
	if(jdata[VERTEX].empty())
	{
		return 2;
	}
	for(int i = 0 ; i< jdata[VERTEX].size(); ++i)
	{
		Vertex vtx;
		if(0!=vtx.convertFromJson(jdata[VERTEX][i]))
		{
			return 2;
		}
		this->vts[i] = vtx;
	}
	if(0!=this->param.convertFromJson(jdata[PARAMS]))
	{
		return 2;
	}
	//scene_name = jdata[SCENE_NAME].asString();
	//map_name = jdata[MAP_NAME].asString();
	region_name = jdata[REGION_NAME].asString();
	region_id = jdata[REGION_ID].asInt();
    region_type = jdata[REGION_TYPE].asInt();
	return 0;
}
int Region::converFromSql(char** sql)
{
	//ROS_ERROR("come");
	region_id = std::atoi(sql[0]);
	//scene_name = sql[1];
	//map_name = sql[2];
	region_name = sql[1];
	region_type = std::atoi(sql[2]);
	std::string vtxs = sql[3];
	//ROS_ERROR("find %d",region_id);

	std::vector<std::string> vecSegTag;
	//ROS_INFO("%s",str.c_str());
	boost::split(vecSegTag, vtxs ,boost::is_any_of("|"));
	//ROS_INFO("size:%d",(int)vecSegTag.size());
	vts.resize(vecSegTag.size());
	for(int i = 0; i < vecSegTag.size(); ++i)
	{
		Vertex vtx;
		vtx.converFromSql(vecSegTag[i]);
		vts[i] = vtx;
	}
	param.converFromSql(sql);
	return 0;

}
void Region::convertToInsertSql(std::string &sql)
{
        char value[2048];
	std::sprintf(value,"%d,'%s',%d,",region_id,region_name.c_str(),region_type);
	sql.append(value);
	//std::stringstream ss;
	//ss << region_id <<",'"<<region_name<<"',"<<region_type<<","<<std::endl;
	//ss >> sql;
	std::string vtxs;
	vtxs.append("'");
	for(int i=0; i < vts.size();i++)
	{
		std::string vex;
		vts[i].convertToSql(vex);
		vtxs.append(vex);
		if(i!=vts.size()-1)
		{
			vtxs.append("|");
		}
	}
	vtxs.append("',");
	sql.append(vtxs);
	std::string pars;
	param.convertToInsertSql(pars);
	sql.append(pars);
	//ROS_INFO("%s",sql.c_str());
}
void Region::convertToUpdateSql(std::string &sql)
{
	std::stringstream ss;
	//ss <<REGION_ID<<"="<<region_id << ","<<REGION_NAME<<"='"<<region_name<<"',"<<REGION_TYPE<<"="<<region_type<<","<<VERTEX<<"="<<std::endl;
	//ss>>sql;
	char value[2048];
	std::sprintf(value,"%s=%d,%s='%s',%s=%d,%s=",REGION_ID.c_str(),region_id,REGION_NAME.c_str(),region_name.c_str(),REGION_TYPE.c_str(),region_type,VERTEX.c_str());
	sql.append(value);	
	std::string vtxs;
	vtxs.append("'");
	for(int i=0; i < vts.size();i++)
	{
		std::string vex;
		vts[i].convertToSql(vex);
		vtxs.append(vex);
		if(i!=vts.size()-1)
		{
			vtxs.append("|");
		}
	}
	vtxs.append("',");
	sql.append(vtxs);
	std::string pars;
	param.convertToUpdateSql(pars);
	sql.append(pars);
	//ROS_INFO("%s",sql.c_str());

}
Param::Param()
{

}
Param::~Param()
{

}
void Param::convertToJson(Json::Value& jdata)const
{
	jdata[MAX_VEL_X] = max_vel_x;
	jdata[ACC_X] = acc_x;
	jdata[MAX_TH] = max_th;
	jdata[ACC_TH] = acc_th;
	jdata[COLLISION_STOP_DIST] = collision_stop_dist;
	jdata[COLLISION_AVOIDANCE] = collision_avoidance;
	jdata[WAITING_TIME] = waiting_time;
	jdata[PATH_TOLORANCE] = path_tolorance;
	jdata[NAV_TYPE] = nav_type;
	jdata[AUDIO_NAME] = audio_name;
	jdata[THRESHOLD] = threshold;
	jdata[INTERVAL_TIME] = interval_time;
	jdata[ENABLE_MAP_REGISTERATION] = enable_map_registeration;
	jdata[MARK_DURATION] = mark_duration;
	jdata[ENABLE_SUPERSONIC] = enable_supersonic;
	jdata[EBALBE_MICROLASER] = enable_microlaser;
	jdata[ENABLE_CAMERA] = enable_camera;
	jdata[ENABLE_IMU] = enable_imu;
	jdata[ENABLE] = enable;
	jdata[PRIORITY] = priority;
}
void Param::convertToNavJson(Json::Value& jdata)const
{

	jdata[MAX_VEL_X] = max_vel_x;
	jdata[ACC_X] = acc_x;
	jdata[MAX_TH] = max_th;
	jdata[ACC_TH] = acc_th;
	jdata[COLLISION_STOP_DIST] = collision_stop_dist;
	jdata[COLLISION_AVOIDANCE] = collision_avoidance;
	jdata[WAITING_TIME] = waiting_time;
	jdata[PATH_TOLORANCE] = path_tolorance;
	jdata[NAV_TYPE] = nav_type;
	jdata[ENABLE] = enable;
	jdata[PRIORITY] = priority;
}
void Param::convertToSensorJson(Json::Value& jdata)const
{
	jdata[ENABLE_SUPERSONIC] = enable_supersonic;
	jdata[EBALBE_MICROLASER] = enable_microlaser;
	jdata[ENABLE_CAMERA] = enable_camera;
	jdata[ENABLE_IMU] = enable_imu;
}
void Param::convertToLocJson(Json::Value& jdata)const
{
	jdata[INTERVAL_TIME] = interval_time;
	jdata[ENABLE_MAP_REGISTERATION] = enable_map_registeration;
	jdata[MARK_DURATION] = mark_duration;

}
void Param::convertToAudioJson(Json::Value& jdata)const
{
	jdata[AUDIO_NAME] = audio_name;
	jdata[INTERVAL_TIME] = interval_time;
}
int Param::convertFromJson(const Json::Value& jdata)
{
	if(!checkJson(jdata,MAX_VEL_X,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,ACC_X,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,MAX_TH,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,ACC_TH,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,COLLISION_STOP_DIST,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,COLLISION_AVOIDANCE,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,WAITING_TIME,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,PATH_TOLORANCE,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,NAV_TYPE,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,AUDIO_NAME,Json::stringValue))
	{
		return 2;
	}
	if(!checkJson(jdata,THRESHOLD,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,INTERVAL_TIME,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,ENABLE_MAP_REGISTERATION,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,MARK_DURATION,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,ENABLE_SUPERSONIC,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,EBALBE_MICROLASER,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,ENABLE_CAMERA,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,ENABLE_IMU,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,ENABLE,Json::intValue))
	{
		return 2;
	}
	if(!checkJson(jdata,PRIORITY,Json::intValue))
	{
		return 2;
	}
	max_vel_x = jdata[MAX_VEL_X].asDouble();
	acc_x = jdata[ACC_X].asDouble();
	max_th = jdata[MAX_TH].asDouble();
	acc_th = jdata[ACC_TH].asDouble();
	collision_stop_dist = jdata[COLLISION_STOP_DIST].asDouble();
	collision_avoidance = jdata[COLLISION_AVOIDANCE].asInt();
	waiting_time = jdata[WAITING_TIME].asDouble();
	path_tolorance = jdata[PATH_TOLORANCE].asDouble();
	nav_type = jdata[NAV_TYPE].asInt();
	audio_name = jdata[AUDIO_NAME].asString();
	threshold = jdata[THRESHOLD].asDouble();
	interval_time = jdata[INTERVAL_TIME].asDouble();
	enable_map_registeration = jdata[ENABLE_MAP_REGISTERATION].asInt();
	mark_duration = jdata[MARK_DURATION].asDouble();
	enable_supersonic = jdata[ENABLE_SUPERSONIC].asInt();
	enable_microlaser = jdata[EBALBE_MICROLASER].asInt();
	enable_camera = jdata[ENABLE_CAMERA].asInt();
	enable_imu = jdata[ENABLE_IMU].asInt();
	enable = jdata[ENABLE].asInt();
	priority = jdata[PRIORITY].asInt();
	return 0;
}
void Param::convertToInsertSql(std::string &sql)
{
	char value[1024];
	std::sprintf(value,"%f,%f,%f,%f,%f,%d,%f,%f,%d,'%s',%f,%f,%d,%f,%d,%d,%d,%d,%d,%d",
			max_vel_x,acc_x,max_th,acc_th,collision_stop_dist,collision_avoidance,waiting_time,path_tolorance,nav_type,
			audio_name.c_str(),interval_time,threshold,enable_map_registeration,mark_duration,enable_supersonic,enable_microlaser,enable_camera,enable_imu,enable,priority);
	sql.append(value);
}
void Param::convertToUpdateSql(std::string &sql)
{
	std::stringstream ss;
	char sq[2048];
	std::sprintf(sq,"%s=%f,%s=%f,%s=%f,%s=%f,%s=%f,%s=%d,%s=%f,%s=%f,%s=%d,%s='%s',%s=%f,%s=%f,%s=%d,%s=%f,%s=%d,%s=%d,%s=%d,%s=%d,%s=%d,%s=%d",MAX_VEL_X.c_str(),max_vel_x,ACC_X.c_str(),acc_x,MAX_TH.c_str(),max_th,ACC_TH.c_str(),acc_th,COLLISION_STOP_DIST.c_str(),collision_stop_dist,COLLISION_AVOIDANCE.c_str(),collision_avoidance,WAITING_TIME.c_str(),waiting_time,PATH_TOLORANCE.c_str(),path_tolorance,NAV_TYPE.c_str(),nav_type,AUDIO_NAME.c_str(),audio_name.c_str(),INTERVAL_TIME.c_str(),interval_time,THRESHOLD.c_str(),threshold,ENABLE_MAP_REGISTERATION.c_str(),enable_map_registeration,MARK_DURATION.c_str(),mark_duration,ENABLE_SUPERSONIC.c_str(),enable_supersonic,EBALBE_MICROLASER.c_str(),enable_microlaser,
ENABLE_CAMERA.c_str(),enable_camera,
ENABLE_IMU.c_str(),enable_imu,
ENABLE.c_str(),enable,PRIORITY.c_str(),priority);
/*	ss<<MAX_VEL_X<<"="<<max_vel_x<<","<<ACC_X<<"="<<acc_x<<","<<MAX_TH<<"="<<max_th<<","<<ACC_TH<<"="<<acc_th<<","<<COLLISION_STOP_DIST<<"="<<collision_stop_dist
			<<","<<COLLISION_AVOIDANCE<<"="<<collision_avoidance<<","<<WAITING_TIME<<"="<<waiting_time<<","<<PATH_TOLORANCE<<"="<<path_tolorance<<","<<NAV_TYPE<<"="<<nav_type
			<<","<<AUDIO_NAME<<"='"<<audio_name<<"',"<<INTERVAL_TIME<<"="<<interval_time<<","<<THRESHOLD<<"="<<threshold<<","<<ENABLE_MAP_REGISTERATION<<"="<<enable_map_registeration
			<<","<<MARK_DURATION<<"="<<mark_duration<<","<<ENABLE_SUPERSONIC<<"="<<enable_supersonic<<","<<EBALBE_MICROLASER<<"="<<enable_microlaser
			<<","<<ENABLE_CAMERA<<"="<<enable_camera<<","<<ENABLE_IMU<<"="<<enable_imu<<","<<ENABLE<<"="<<"enable"<<","<<PRIORITY<<"="<<priority<<std::endl;
*/	sql = sq;
//	ROS_ERROR("sql:%s",sql.c_str());
}
int Param::converFromSql(char** sql)
{
	max_vel_x = std::atof(sql[4]);
	acc_x = std::atof(sql[5]);
	max_th = std::atof(sql[6]);
	acc_th = std::atof(sql[7]);
	collision_stop_dist = std::atof(sql[8]);
	collision_avoidance = std::atoi(sql[9]);
	waiting_time = std::atof(sql[10]);
	path_tolorance = std::atof(sql[11]);
	nav_type = std::atoi(sql[12]);
	audio_name = sql[13];
	interval_time = std::atof(sql[14]);
	threshold= std::atof(sql[15]);
	enable_map_registeration = std::atoi(sql[16]);
	mark_duration = std::atof(sql[17]);
	enable_supersonic = std::atoi(sql[18]);
	enable_microlaser = std::atoi(sql[19]);
	enable_camera = std::atoi(sql[20]);
	enable_imu = std::atoi(sql[21]);
	enable = std::atoi(sql[22]);
	priority = std::atoi(sql[23]);

	return 0;
}
Vertex::Vertex()
{

}
Vertex::~Vertex()
{

}
void Vertex::convertToJson(Json::Value& jdata)
{
	jdata[X] = x;
	jdata[Y] = y;
	jdata[ID] = id;
}
int Vertex::convertFromJson(const Json::Value& jdata)
{
	if(!checkJson(jdata,X,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,Y,Json::realValue))
	{
		return 2;
	}
	if(!checkJson(jdata,ID,Json::intValue))
	{
		return 2;
	}
	x = jdata[X].asDouble();
	y = jdata[Y].asDouble();
	id = jdata[ID].asInt();
	return 0;
}

void Vertex::convertToSql(std::string& sql)
{
	std::stringstream ss;
	ss << id<<","<< x << "," << y  << std::endl;
	ss>>sql;
}
int Vertex::converFromSql(std::string & sql)
{
	std::vector<std::string> vecSegTag;
	//ROS_INFO("%s",str.c_str());
	boost::split(vecSegTag, sql ,boost::is_any_of(","));
	//ROS_INFO("size:%d",(int)vecSegTag.size());
	id = atoi(vecSegTag[0].c_str());
	x = atof(vecSegTag[1].c_str());
	y = atof(vecSegTag[2].c_str());
	return 0;
}
Field::Field(std::string field_name, int type):field_name(field_name),type(type)
{

}
void Field::createFieldSql(std::string &sql)const
{
	sql.append(this->field_name);
	switch(type)
	{
	case 0:sql.append(" REAL");break;
	case 1:sql.append(" INT");break;
	case 2:sql.append(" TEXT");break;
	default:sql.append(" TEXT");
	}

	sql.append(" NOT NULL");
}
void Field::createInsertSql(std::string &sql)const
{
	sql.append(this->field_name);
}
void Table::createInsertSql(std::string &sql)const
{
	sql.clear();
	sql.append("INSERT INTO ");
	sql.append(table_name);
	sql.append("(");
	for(std::vector<Field>::const_iterator field_it = fields.begin(); field_it != fields.end();++field_it)
	{
		field_it->createInsertSql(sql);
		if(field_it+1 != fields.end())
		{
			sql.append(",");
		}

	}
	sql.append(")");
}
void Table::createTableSql(std::string &sql)const
{
	sql.clear();
	sql.append("CREATE TABLE ");
	sql.append(table_name);
	sql.append("(");
	for(std::vector<Field>::const_iterator field_it = fields.begin(); field_it != fields.end();++field_it)
	{
		field_it->createFieldSql(sql);
		if(field_it+1 != fields.end())
		{
			sql.append(",");
		}

	}
	sql.append(");");
}

