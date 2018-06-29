/*
 * json_string_variables.h
 *
 *  Created on: Apr 6, 2017
 *      Author: tomzhang
 */

#ifndef MAP_SERVER_INCLUDE_MAP_SERVER_JSON_STRING_VARIABLES_H_
#define MAP_SERVER_INCLUDE_MAP_SERVER_JSON_STRING_VARIABLES_H_



static const std::string PUB_NAME = "pub_name";

static const std::string SUB_NAME = "sub_name";

static const std::string CMD_MAP_LIST_GET = "map_list_get";

static const std::string CMD_MAP_CURRENT_GET = "map_current_get";

static const std::string CMD_MAP_RENAME = "map_rename";

static const std::string CMD_MAP_SCENE_RENAME = "map_scene_rename";

static const std::string CMD_MAP_DELETE = "map_delete";

static const std::string CMD_SCENE_DELETE = "map_scene_delete";

static const std::string CMD_MAP_SAVE = "map_save";

static const std::string CMD_MAP_LOAD = "map_load";

static const std::string CMD_MAP_EDIT = "map_edit";

static const std::string MAP_EDIT_COMPLETE = "map_edit_complete";

static const std::string CMD_MAP_DEFAULT_SET ="map_default_set";

static const std::string CMD_MAP_DEFAULT_GET ="map_default_get";

static const std::string CMD_POINT_ADD = "point_add";

static const std::string CMD_POINT_DELETE = "point_delete";

static const std::string CMD_POINT_QUERY = "point_query";

static const std::string CMD_POINT_EDIT = "point_edit";

static const std::string CMD_POINT_LOAD = "point_load";

static const std::string CMD_VIRTUAL_WALL_LOAD = "virtual_wall_load";

static const std::string CMD_VIRTUAL_WALL_ADD = "virtual_wall_add";

static const std::string CMD_VIRTUAL_WALL_DELETE = "virtual_wall_delete";

static const std::string CMD_LABEL_SAVE = "label_save";

static const std::string CMD_LABEL_DELETE = "label_delete";

static const std::string DATA = "data";

static const std::string MSG = "msg";

static const std::string ERROR_CODE = "error_code";

static const std::string ORIGIN_NAME = "origin_name";

static const std::string NEW_NAME = "new_name";

static const std::string ORIGIN_SCENE_NAME = "origin_scene_name";

static const std::string NEW_SCENE_NAME = "new_scene_name";

static const std::string SCENE_NAME = "scene_name";

static const std::string MAP_NAME = "map_name";

static const std::string POINT_NAME = "point_name";

static const std::string POINT_ALIAS = "point_alias";

static const std::string MAP_NAMES = "mapNames";

static const std::string Z = "z";

static const std::string X = "x";

static const std::string Y = "y";

static const std::string TH = "th";

static const std::string NEW_X = "new_x";

static const std::string NEW_Y = "new_y";

static const std::string NEW_TH = "new_th";

static const std::string POINT_TYPE = "point_type";

static const std::string POINTS = "points";

static const std::string NEW_POINT_NAME = "new_point_name";

static const std::string NEW_POINT_ALIAS = "new_point_alias";

static const std::string NEW_POINT_TYPE = "new_point_type";

static const std::string ID = "id";

static const std::string TAG = "tag";

static const std::string RADIUS = "radius";

static const std::string CIRCLES = "circles";

static const std::string CIRCLE = "circle";

static const std::string POLYGONS = "polygons";

static const std::string POLYGON = "polygon";

static const std::string LINESEGMENTS = "linesegments";

static const std::string START_X = "start_x";

static const std::string START_Y = "start_y";

static const std::string END_X = "end_x";

static const std::string END_Y = "end_y";

static const std::string ORIGIN_X = "origin_x";

static const std::string ORIGIN_Y = "origin_y";

static const std::string ORIGIN = "origin";

static const std::string CURRENT_MAX_ID = "current_max_id";


const static std::string CMD_REGIONS_SCENE_GET = "regions_scene_get";
const static std::string CMD_REGIONS_MAP_GET = "regions_map_get";
const static std::string CMD_REGION_CREATE = "region_create";
const static std::string CMD_REGION_REMOVE = "region_remove";
const static std::string CMD_REGION_PARAMS_GET = "region_params_get";
const static std::string CMD_REGION_MODIFY = "region_modify";
const static std::string REGIONS = "regions";
const static std::string REGION_NAME = "region_name";
const static std::string REGION_TYPE = "region_type";
const static std::string REGION_ID ="region_id";
const static std::string VERTEX = "vertex";
const static std::string PARAMS = "params";
const static std::string MAX_VEL_X = "max_vel_x";
const static std::string ACC_X = "max_acc_x";
const static std::string MAX_TH = "max_vel_th";
const static std::string ACC_TH = "max_acc_th";
const static std::string COLLISION_STOP_DIST = "collision_stop_dist";
const static std::string COLLISION_AVOIDANCE = "collision_avoidance";
const static std::string WAITING_TIME = "waiting_time";
const static std::string PATH_TOLORANCE = "path_tolorance";
const static std::string NAV_TYPE = "nav_type";
const static std::string ENABLE = "enable";
const static std::string PRIORITY = "priority";
const static std::string AUDIO_NAME="audio_name";
const static std::string THRESHOLD="threshold";
const static std::string INTERVAL_TIME="interval_time";
const static std::string ENABLE_MAP_REGISTERATION="enable_map_registeration";
const static std::string MARK_DURATION="mark_duration";
const static std::string ENABLE_SUPERSONIC="enable_supersonic";
const static std::string EBALBE_MICROLASER="enable_microlaser";
const static std::string ENABLE_CAMERA="enable_camera";
const static std::string ENABLE_IMU="enable_imu";

const static unsigned int TYPE_LINE = 0;
const static unsigned int TYPE_CURVE = 1;

static const std::string FIXPATH_PARAM_ADD="fixpath_param_add";
static const std::string FIXPATH_QUERY="fixpath_query";
static const std::string FIXPATH_MAP_QUERY="fixpath_map_query";
static const std::string FIXPATH_PATH_QUERY="fixpath_path_query";
static const std::string FIXPATH_DELETE="fixpath_delete";
static const std::string FIXPATH_EDIT="fixpath_edit";
static const std::string FIXPATH_FILE_QUERY="fixpath_file_query";

static const std::string RECORD_PATH_START="record_path_start";
static const std::string RECORD_PATH_END="record_path_end";

static const std::string INSERT_DATA="INSERT INTO PATH (ID,TYPE,POINT_INDEX,X,Y,TH)";
static const std::string CREATE_PATH_TABLE="CREATE TABLE PATH ("  \
        "ID             INT     NOT NULL," \
        "TYPE           INT     NOT NULL," \
        "POINT_INDEX    INT     NOT NULL," \
        "X         REAL         NOT NULL," \
        "Y         REAL         NOT NULL," \
        "TH        REAL );";
static const std::string INSERT_OP = "INSERT INTO PATH (ID,TYPE,POINT_INDEX,X,Y,TH) " "VALUES (";
static const std::string INQUERY_OP = "SELECT * FROM PATH";
static const std::string DELETE_OP = "DELETE FROM PATH WHERE ID = ";




//const static std::string CREATE_REGION_TABLE="CREATE TABLE IF NOT EXISTS REGIONS("  \
//        "REGION_ID             INT     NOT NULL," \
//        "SCENE_NAME           INT     NOT NULL," \
//        "MAP_NAME            TEXT    NOT NULL," \
//        "REGION_NAME         TEXT         NOT NULL," \
//        "VERTEX    TEXT     NOT NULL," \
//        "MAX_VEL_X         REAL         NOT NULL," \
//        "ACC_X         REAL         NOT NULL," \
//        "MAX_TH         REAL         NOT NULL," \
//        "ACC_TH         REAL         NOT NULL," \
//        "COLLISION_STOP_DIST         REAL         NOT NULL," \
//        "COLLISION_AVOIDANCE         INT         NOT NULL," \
//        "WAITING_TIME         REAL         NOT NULL," \
//        "PATH_TOLORANCE         REAL         NOT NULL," \
//        "NAV_TYPE         INT         NOT NULL," \
//        "ENABLE         INT         NOT NULL," \
//        "PRIORITY         INT         NOT NULL);";
const static std::string INSERT = "INSERT INTO regions (region_id,region_name,region_type,vertex,"\
									"max_vel_x,max_acc_x,max_vel_th,max_acc_th,collision_stop_dist,collision_avoidance,"\
									"waiting_time,path_tolorance,nav_type,audio_name,interval_time,threshold,"\
									"enable_map_registeration,mark_duration,enable_supersonic,enable_microlaser,"\
									"enable_camera,enable_imu,enable,priority)";

bool checkJson(const Json::Value& jdata, const std::string& tag, Json::ValueType type);


#endif /* MAP_SERVER_INCLUDE_MAP_SERVER_JSON_STRING_VARIABLES_H_ */
