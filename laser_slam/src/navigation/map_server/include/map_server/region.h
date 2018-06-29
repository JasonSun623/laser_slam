/*
 * param.h
 *
 *  Created on: Aug 31, 2017
 *      Author: tomzhang
 */

#ifndef INCLUDE_MAP_SERVER_REGION_H_
#define INCLUDE_MAP_SERVER_REGION_H_
#include "json/json.h"
namespace map_server
{
struct Field
{
public:
    Field(std::string field_name, int type);
	void createFieldSql(std::string &sql)const;
	void createInsertSql(std::string &sql)const;
	std::string field_name;
	int type;
};

struct Table
{
public:
	void createTableSql(std::string &sql)const;
	void createInsertSql(std::string &sql)const;
	//void addFieldSql(std::string &sql)const;
	std::string table_name;
	std::vector<Field> fields;
};

class Param
{
public:
	Param();
	~Param();
    void convertToJson(Json::Value& jdata)const;
    void convertToNavJson(Json::Value& jdata)const;
    void convertToLocJson(Json::Value& jdata)const;
    void convertToSensorJson(Json::Value& jdata)const;
    void convertToAudioJson(Json::Value& jdata)const;
    int convertFromJson(const Json::Value& jdata);
    void convertToInsertSql(std::string& sql);
    void convertToUpdateSql(std::string& sql);
    int converFromSql( char** sql);
	double max_vel_x,acc_x,max_th,acc_th,collision_stop_dist,waiting_time,path_tolorance,threshold,interval_time,mark_duration;
	std::string audio_name;
	int collision_avoidance,nav_type,enable,priority,enable_map_registeration,
	enable_supersonic,enable_microlaser,enable_camera,enable_imu;

};

class Vertex
{
public:
	Vertex();
	~Vertex();
    void convertToJson(Json::Value& jdata);
    int convertFromJson(const Json::Value& jdata);
    void convertToSql(std::string& sql);
    int converFromSql( std::string & sql);
	int id;
    double x,y;

};
class Region
{
public:
	Region();
	~Region();
	void convertToJson(Json::Value& jdata);
	int convertFromJson(const Json::Value& jdata);
    void convertToInsertSql(std::string& sql);
    void convertToUpdateSql(std::string& sql);
    int converFromSql( char** sql);
    const Param& getParam()const{return param;};
    int region_type;
	std::string region_name;
	int region_id;
	Param param;
	std::vector<Vertex> vts;

};
}
#endif /* INCLUDE_MAP_SERVER_REGION_H_ */
