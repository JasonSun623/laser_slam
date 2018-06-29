/*
 * fixpath_operations.hpp
 *
 *  Created on: Oct 30, 2017
 *      Author: Gavin, tomzhang
 */


/******
 * line type:
 * 0:  line  , save all line points to database,and feedback all points to fixpath global package,but
 *          send back start and end points to upper controller
 * 1:  bezier curve, save bezier key points to database,and feedback all points to fixpath global package,but
 *          send back all bezier key poihts to upper controller
 * 2:  recorded line, save all points to database, and feedback all points to fixpath global package,and
 *          send back all points to upper controller by id query(if query all,just send back start and end points).
 */
#ifndef MAP_SERVER_INCLUDE_MAP_SERVER_FIXPATH_OPERATIONS_HPP_
#define MAP_SERVER_INCLUDE_MAP_SERVER_FIXPATH_OPERATIONS_HPP_
#include "map_server/map_server_mrobot.h"
using namespace map_server;
static int database_callback(void *NotUsed, int argc, char **argv, char **azColName);
static int database_callback(void *NotUsed, int argc, char **argv, char **azColName)
{
	//FixpathCtrl* ptr = (FixpathCtrl *)NotUsed;
	std::vector<path_t>* ptr = (std::vector<path_t> *)NotUsed;

	std::string id = argv[0];
	std::string type = argv[1];
	std::string point_index = argv[2];
	std::string x = argv[3];
	std::string y = argv[4];
	std::string th = argv[5];
	for(std::vector<path_t>::iterator it= ptr->begin();it != ptr->end(); it++)
	{
		if(it->id == std::atoi(id.c_str()))
		{
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id ="map";
			pose.pose.position.x = std::atof(x.c_str());
			pose.pose.position.y = std::atof(y.c_str());
			pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::atof(th.c_str()));
			it->path.push_back(pose);
			it->type = std::atoi(type.c_str());

			return 0;
		}
	}
	path_t p;
	p.id = std::atoi(id.c_str());
	p.type = std::atoi(type.c_str());
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = std::atof(x.c_str());
	pose.pose.position.y = std::atof(y.c_str());
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::atof(th.c_str()));
	p.path.push_back(pose);
	ptr->push_back(p);

	return 0;
}

void MapServer::fixpath_param_add(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	Json::Value data;
	path_t p;
	data = (*jptr)[DATA];
	p.scene_name = data[SCENE_NAME].asString();
	p.map_name = data[MAP_NAME].asString();
	p.id = find_unuse_path_id(p.scene_name, p.map_name);
	//ROS_ERROR("scene_name:%s, map_name:%s",p.scene_name.c_str(),p.map_name.c_str());

	if(TYPE_LINE == data["type"].asInt())
	{
		bool success = get_line_start_end(data,p);
		if(success)
		{
			success = create_line_path(p.start, p.end,p.path);
			if(success)
			{
				save_path_database(p);
				err = 0;
				dataArray[ID] = p.id;
				msg_info = "success";
			}
			else
			{
				err = 1;
				msg_info = "create line failed";
			}
		}
		else
		{
			err = 2;
			msg_info = "get line failed";
		}
	}
    else if(TYPE_CURVE == data["type"].asInt())
    {
        //1: bezier curve
        bool success = get_bezier_curve_point(data,p);
        if(success)
        {
            if(0 == ((p.path.size()-1)%3))
            {
                save_path_database(p);
                dataArray[ID] = p.id;
                err= 0;
                msg_info = "success";
            }
            else
            {
                err= 1;
                msg_info = "create curve failed";
            }
        }
        else
        {
            err= 2;
            msg_info = "get curve failed";
        }
    }
	else
	{
		err = 3;
		msg_info = "json invalid!";
	}

}
//query all path in scene.
void MapServer::fixpath_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	//ROS_INFO("query all scene fixpath");
	//query all fixpath;
	Json::Value data;
	data = (*jptr)[DATA];
	std::string scene_name = data[SCENE_NAME].asString();
	std::vector<point_t> points;
	scene sce;
	sce.scene_name = scene_name;
	map_list_manager_->traverse_folder(sce);
	std::vector<std::string> map_list = sce.map_list;
	//step 1 get all goal points;
	if(false == get_all_points(map_list,scene_name,points))
	{
		err = 1;
		msg_info = "query all points operation failed!";
		return ;
	}
    //step 2 get all path.
	Json::Value path;
	if(true == get_fixpath_info_by_scene(map_list,scene_name,path,points))
	{
		//step 3 return message;
		dataArray["paths"] = path;
		dataArray[SCENE_NAME] = scene_name;
		err = 0;
		msg_info = "success!";
	}
	else
	{
		err = 2;
		msg_info = "get scene path info failed!";
	}
	//ROS_INFO("query line fix path");
}
//query all path in map
void MapServer::fixpath_map_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    //ROS_INFO("query all map fixpath");
    Json::Value data;
    data = (*jptr)[DATA];
    std::string scene_name = data[SCENE_NAME].asString();
    std::string map_name = data[MAP_NAME].asString();
    std::vector<point_t> points;
    scene sce;
    sce.scene_name = scene_name;
    map_list_manager_->traverse_folder(sce);
    std::vector<std::string> map_list = sce.map_list;
    //step 1 get all goal points;
    if(false == get_all_points(map_list,scene_name,points))
    {
        err = 1;
        msg_info = "query all points operation failed!";
        return ;
    }
    //step 2 get all path.
    Json::Value path;
    std::vector<std::string> map;
    map.push_back(map_name);
    if(true == get_fixpath_info_by_scene(map,scene_name,path,points))
    {
        //step 3 return message;
        dataArray["paths"] = path;
        dataArray[SCENE_NAME] = scene_name;
        dataArray[MAP_NAME] = map_name;
        err = 0;
        msg_info = "success!";
    }
    else
    {
        err = 2;
        msg_info = "get scene path info failed!";
    }
    //ROS_INFO("query line fix path");
}
//query path info
void MapServer::fixpath_path_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    //ROS_INFO("query all map fixpath");
    Json::Value data;
    data = (*jptr)[DATA];
    std::string scene_name = data[SCENE_NAME].asString();
    std::string map_name = data[MAP_NAME].asString();
    unsigned int id = data[ID].asInt();
    std::vector<point_t> points;
    scene sce;
    sce.scene_name = scene_name;
    map_list_manager_->traverse_folder(sce);
    std::vector<std::string> map_list = sce.map_list;
    //step 1 get all goal points;
    if(false == get_all_points(map_list,scene_name,points))
    {
        err = 1;
        msg_info = "query all points operation failed!";
        return ;
    }
    //step 2 get all path.
    Json::Value path;
    if(true == get_fixpath_info_by_id(id,map_name,scene_name,path,points))
    {
        //step 3 return message;
        dataArray["paths"] = path;
        dataArray[SCENE_NAME] = scene_name;
        err = 0;
        msg_info = "success!";
    }
    else
    {
        err = 2;
        msg_info = "get scene path info failed!";
    }
    //ROS_INFO("query line fix path");
}

void MapServer::fixpath_file_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	//ROS_INFO("query all scene fixpath");
	//query all fixpath;
	Json::Value data;
	data = (*jptr)[DATA];
	std::string scene_name = data[SCENE_NAME].asString();
	std::vector<point_t> points;
	scene sce;
	sce.scene_name = scene_name;
	map_list_manager_->traverse_folder(sce);
	std::vector<std::string> map_list = sce.map_list;
	//step 1 get all goal points;
	if(false == get_all_points(map_list,scene_name,points))
	{
		err = 1;
		msg_info = "query all points operation failed!";
		return ;
	}
	Json::Value path;
	if(true == get_fixpath_info_by_scene(map_list,scene_name,path,points,true))
	{
		//step 3 return message;
		//dataArray["paths"] = path;
		Json::Value pathes;
		pathes["paths"] = path;
		pathes[SCENE_NAME] = scene_name;
		dataArray[SCENE_NAME] = scene_name;
		Json::FastWriter writer;
		if(write_string_to_file(scene_name,writer.write(pathes),"pathes.txt"))
		{
			err = 0;
			msg_info = "success!";
		}
		else
		{
			err = 3;
			msg_info = "failed!";
		}
	}
	else
	{
		err = 2;
		msg_info = "get scene path info failed!";
	}
	//ROS_INFO("query line fix path");
}
bool MapServer::write_string_to_file(std::string scene_name,const std::string & text, std::string file_name)
{
	std::string file_path = config_map_path_ + scene_name + "/"+file_name;
	std::ofstream text_file(file_path.c_str(), std::ios::trunc);
	if(!text_file)
	{
		ROS_ERROR("open file %s failed!", file_path.c_str());
		return false;
	}
	text_file << text;
	text_file.close();
	
}
void MapServer::fixpath_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	ROS_INFO("fixpath delete");
	Json::Value data;
	data = (*jptr)[DATA];
	path_t p;
	p.scene_name = data[SCENE_NAME].asString();
	p.map_name = data[MAP_NAME].asString();
	p.id = data[ID].asInt();

	if(delete_fixpath(p))
	{
		err = 0;
		msg_info = "delete success";
	}
	else
	{
		err = 1;
		msg_info = "delete failed";
	}

}


void MapServer::fixpath_edit(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	ROS_INFO("fixpath edit");

    Json::Value data = (*jptr)[DATA];    
    std::string scene_name = data[SCENE_NAME].asString();
    std::string map_name = data[MAP_NAME].asString();
    unsigned int id = data[ID].asInt();


    path_t path_delete;
    path_delete.scene_name = scene_name;
    path_delete.map_name = map_name;
    path_delete.id = id;

    if(!delete_fixpath(path_delete))
    {
        err = 1;
        msg_info = "delete failed";
        return;
    }

    path_t path_add;
    path_add.scene_name = scene_name;
    path_add.map_name = map_name;
    path_add.id = id;

    switch(data["type"].asInt())
    {
        case TYPE_LINE:
            if(!get_line_start_end(data, path_add))
            {
                err = 2;
                msg_info = "get line failed";
                return;
            }
            if(!create_line_path(path_add.start, path_add.end, path_add.path))
            {
                err = 3;
                msg_info = "create line failed";
                return;
            }
            break;
        case TYPE_CURVE:

            if(!get_bezier_curve_point(data, path_add))
            {
                err = 4;
                msg_info = "get curve failed";
                return;
            }

            if(((path_add.path.size() - 1) % 3) != 0)
            {
                err = 5;
                msg_info = "create curve failed";
                return;
            }
            break;
        default:
            break;
    }
    
    save_path_database(path_add);
    err = 0;
    dataArray[ID] = path_add.id;
    msg_info = "success";
}


void MapServer::fixpath_record_start(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	ROS_INFO("record path start");
	if(record_path.size())
	{
		record_path.clear();
	}
	record_path_start = true;
	err = 0;;
	msg_info = "record start!";
}
void MapServer::fixpath_record_finish(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	ROS_INFO("record path end");
	Json::Value data;
	data = (*jptr)[DATA];
	path_t p;
	p.scene_name = data[SCENE_NAME].asString();
	p.map_name = data[MAP_NAME].asString();
	p.id = find_unuse_path_id(p.scene_name,p.map_name);
	ROS_ERROR("scene_name:%s, map_name:%s",p.scene_name.c_str(),p.map_name.c_str());

	record_path_start = false;
	if(record_path.size() < 1)
	{
		err = 1;
		msg_info = "record path failed";
		return;
	}
	//save record_path to file;
	p.type = 2;
	p.start = record_path[0];
	p.end = record_path[record_path.size()-1];
	p.path = record_path;
	if(save_path_database(p))
	{
		publish_fixpath(record_path);
		err = 0;
		msg_info = "record path success";
	}
	else
	{
		err = 1;
		msg_info = "path save failed";
	}


}
bool MapServer::create_line_path(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal,std::vector<geometry_msgs::PoseStamped>& global_plan)
{
	int i = 1;
	int size = 0;
	double dist = 0;
	double g_deta_x = 0.0;
	double g_deta_y = 0.0;
	double deta_x = 0;
	double deta_y = 0;
	geometry_msgs::PoseStamped path_point;
	double dist_xy_tollerance = 0.05;
	double dist_th_tollerance = 0.15;
	//std::vector<geometry_msgs::PoseStamped> global_plan;
	geometry_msgs::Quaternion quaternion;

	ROS_ERROR("create_line_path");
	path_point = start;
	g_deta_x = goal.pose.position.x - path_point.pose.position.x;
	g_deta_y = goal.pose.position.y - path_point.pose.position.y;
	dist = line_dist(path_point.pose.position.x,path_point.pose.position.y,goal.pose.position.x,goal.pose.position.y);
	double start_th = tf::getYaw(start.pose.orientation);
	double goal_th = tf::getYaw(goal.pose.orientation);
	double theta = adjust_th(goal_th - start_th);

	ROS_ERROR("dist:%.3f, theta:%.3f,start_th:%.3f,goal_th:%.3f",dist,theta,start_th,goal_th);
	if(fabs(dist) < dist_xy_tollerance)
	{
		if(fabs(theta) < dist_th_tollerance)
		{
			ROS_ERROR("create_line_path 1");
			return false;
		}
		else
		{
			size = ceil((unsigned int)(fabs(theta) / dist_th_tollerance));
			double d_th = dist_th_tollerance;
			if(theta < 0)
			{
				d_th = -dist_th_tollerance;
			}
			i = 0;
			while(i < size)
			{
				//path_point.pose.position.x = path_point.pose.position.x;
				//path_point.pose.position.y = path_point.pose.position.y;
				path_point.pose.orientation = tf::createQuaternionMsgFromYaw(start_th + i*d_th);
				i++;
				global_plan.push_back(path_point);
			}
			global_plan.push_back(goal);
		}
	}
	else
	{
		size = ceil(dist / dist_xy_tollerance);
		if(fabs(g_deta_y) < 0.0001)
		{
			if(g_deta_x > 0)
				theta = 0;
			else
				theta = M_PI;

			deta_x = dist_xy_tollerance;
			deta_y = 0;
		}
		else if(fabs(g_deta_x) < 0.0001)
		{
			if(g_deta_y > 0)
				theta = M_PI/2;
			else
				theta = -M_PI/2;

			deta_x = 0;
			deta_y = dist_xy_tollerance;
		}
		else
		{
			//theta = cal_th(start, goal);
			theta = atan2(goal.pose.position.y - start.pose.position.y,goal.pose.position.x - start.pose.position.x);
			//theta = adjust_th(theta);

			deta_x = dist_xy_tollerance * cos(theta);
			deta_y = dist_xy_tollerance * sin(theta);
		}

		quaternion = tf::createQuaternionMsgFromYaw(theta);

		i = 1;
		global_plan.push_back(start);
		while(i < size)
		{
			path_point.pose.position.x = start.pose.position.x + i*deta_x;
			path_point.pose.position.y = start.pose.position.y + i*deta_y;
			path_point.pose.orientation = quaternion;
			ROS_ERROR("%.3f %.3f %.3f",path_point.pose.position.x,path_point.pose.position.y,theta);
			global_plan.push_back(path_point);
			i++;
		}
		global_plan.push_back(goal);
	}

	//save to database
	publish_fixpath(global_plan);
	return true;
}
void MapServer::publish_fixpath(const std::vector<geometry_msgs::PoseStamped>& path)
{
	nav_msgs::Path gui_path;
	gui_path.poses.resize(path.size());

	if(!path.empty())
	{
		gui_path.header.frame_id = path[0].header.frame_id;
		gui_path.header.stamp = path[0].header.stamp;
	}

	for(unsigned int i=0; i < path.size(); i++){
		gui_path.poses[i] = path[i];
	}

	fixpath_plan_pub_.publish(gui_path);
}


int MapServer::find_unuse_path_id(std::string scene_name, std::string map_name)
{
	std::vector<path_t> paths;
	bool flag = get_fixpath_info_by_map(scene_name,map_name,paths);
	if(!flag)
	{
		ROS_ERROR("get fixpath info failed!");
		return 0;
	}
	if(0 == paths.size())
	{
		return 0;
	}
	for(int i = 0;i < paths.size();)
	{
		std::vector<path_t>::iterator it=paths.begin();
		for(;it!= paths.end();it++)
		{
			if(i == it->id)
			{
				i++;
				break;
			}
		}
		if(it == paths.end())
		{
			return i;
		}
	}
	return paths.size();
}
bool MapServer::get_line_start_end(const Json::Value &data,path_t &p)
{
	//get start point;
	p.type = 0;//linear
	Json::Value start_point = data["start"];
	if((start_point[TH].isDouble() || start_point[TH].isInt()) && (start_point[X].isDouble() || start_point[X].isInt()) && (start_point[Y].isDouble() || start_point[Y].isInt()))
	{
		p.start.header.frame_id = "map";
		p.start.header.stamp = ros::Time::now();
		double yaw = 0.0;
		double x = 0.0;
		double y = 0.0;
		if(start_point[TH].isDouble())
		{
			yaw = start_point[TH].asDouble();
		}
		else
		{
			yaw = start_point[TH].asInt();
		}
		if(start_point[X].isDouble())
		{
			x = start_point[X].asDouble();
		}
		else
		{
			x = start_point[X].asInt();
		}
		if(start_point[Y].isDouble())
		{
			y = start_point[Y].asDouble();
		}
		else
		{
			y = start_point[Y].asInt();
		}
		p.start.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		p.start.pose.position.x = x;
		p.start.pose.position.y = y;
	}
	else
	{
		return false;
	}
	//get goal point;
	Json::Value end_point = data["end"];
	if((end_point[TH].isDouble() || end_point[TH].isInt()) && (end_point[X].isDouble() || end_point[X].isInt()) && (end_point[Y].isDouble() || end_point[Y].isInt()))
	{
		p.end.header.frame_id = "map";
		p.end.header.stamp = ros::Time::now();
		double yaw = 0.0;
		double x = 0.0;
		double y = 0.0;
		if(end_point[TH].isDouble())
		{
			yaw = end_point[TH].asDouble();
		}
		else
		{
			yaw = end_point[TH].asInt();
		}
		if(end_point[X].isDouble())
		{
			x = end_point[X].asDouble();
		}
		else
		{
			x = end_point[X].asInt();
		}
		if(end_point[Y].isDouble())
		{
			y = end_point[Y].asDouble();
		}
		else
		{
			y = end_point[Y].asInt();
		}
		p.end.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		p.end.pose.position.x = x;
		p.end.pose.position.y = y;
	}
	else
	{
		return false;
	}
	return true;
}

bool MapServer::get_bezier_curve_point(const Json::Value &data, path_t &p)
{
    geometry_msgs::PoseStamped point;
    p.type = 1;//curve
    Json::Value start_point = data["start"];
    if((start_point["th"].isDouble() || start_point["th"].isInt()) && (start_point["x"].isDouble() || start_point["x"].isInt()) && (start_point["y"].isDouble() || start_point["y"].isInt()))
    {
        p.start.header.frame_id = "map";
        p.start.header.stamp = ros::Time::now();
        double yaw = 0.0;
        double x = 0.0;
        double y = 0.0;
        if(start_point["th"].isDouble())
        {
            yaw = start_point["th"].asDouble();
        }
        else
        {
            yaw = start_point["th"].asInt();
        }
        if(start_point["x"].isDouble())
        {
            x = start_point["x"].asDouble();
        }
        else
        {
            x = start_point["x"].asInt();
        }
        if(start_point["y"].isDouble())
        {
            y = start_point["y"].asDouble();
        }
        else
        {
            y = start_point["y"].asInt();
        }
        p.start.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        p.start.pose.position.x = x;
        p.start.pose.position.y = y;
    }
    else
    {
        return false;
    }
    ROS_INFO("calculate start ok!");
    //get goal point;
    Json::Value end_point = data["end"];
    if((end_point["th"].isDouble() || end_point["th"].isInt()) && (end_point["x"].isDouble() || end_point["x"].isInt()) && (end_point["y"].isDouble() || end_point["y"].isInt()))
    {
        p.end.header.frame_id = "map";
        p.end.header.stamp = ros::Time::now();
        double yaw = 0.0;
        double x = 0.0;
        double y = 0.0;
        if(end_point["th"].isDouble())
        {
            yaw = end_point["th"].asDouble();
        }
        else
        {
            yaw = end_point["th"].asInt();
        }
        if(end_point["x"].isDouble())
        {
            x = end_point["x"].asDouble();
        }
        else
        {
            x = end_point["x"].asInt();
        }
        if(end_point["y"].isDouble())
        {
            y = end_point["y"].asDouble();
        }
        else
        {
            y = end_point["y"].asInt();
        }
        p.end.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        p.end.pose.position.x = x;
        p.end.pose.position.y = y;
    }
    else
    {
        return false;
    }

    ROS_INFO("calculate start and end ok!");
    //get all points
    Json::Value points = data["points"];
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    for(Json::Value::iterator it = points.begin(); it != points.end(); it++)
    {
        if(((*it)["x"].isDouble() || (*it)["x"].isInt()) && ((*it)["y"].isDouble() || (*it)["y"].isInt()))
        {
            if((*it)["x"].isDouble())
            {
                point.pose.position.x = (*it)["x"].asDouble();
            }
            else
            {
                point.pose.position.x = (*it)["x"].asInt();
            }
            if((*it)["y"].isDouble())
            {
                point.pose.position.y = (*it)["y"].asDouble();
            }
            else
            {
                point.pose.position.y = (*it)["y"].asInt();
            }
            double yaw = 0.0;
            if((*it)["th"].isDouble())
            {
                yaw = (*it)["th"].asDouble();
            }
            else
            {
                yaw = (*it)["th"].asInt();
            }
            point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            p.path.push_back(point);
        }
        else
        {
            return false;
        }
    }
    ROS_INFO("calculate all points ok!");

    return true;
}

bool MapServer::create_bezier_curve_path(std::vector<geometry_msgs::PoseStamped> &path)
{

    std::vector<geometry_msgs::PoseStamped> points = path;
    geometry_msgs::PoseStamped temp;
    path.clear();
    if((points.size()-1)%3)
    {
        return false;
    }
    int i = 0;

    temp.header.frame_id = points[0].header.frame_id;
    temp.header.stamp = points[0].header.stamp;
    while(i < points.size()-3)
    {
        double t = 0.0;
        double x0=points[i].pose.position.x;
        double x1=points[i+1].pose.position.x;
        double x2=points[i+2].pose.position.x;
        double x3=points[i+3].pose.position.x;
        double y0=points[i].pose.position.y;
        double y1=points[i+1].pose.position.y;
        double y2=points[i+2].pose.position.y;
        double y3=points[i+3].pose.position.y;
        double xitem0 = x1 - x0;
        double xitem1 = 2*x0 - 4*x1 + 2*x2;
        double xitem2 = x3 - x0 + 3*x1 - 3*x2;
        double yitem0 = y1 - y0;
        double yitem1 = 2*y0-4*y1+2*y2;
        double yitem2 = y3 - y0 + 3*y1 - 3*y2;
        double len = 0.15;
        path.push_back(points[i]);
        ROS_ERROR("x,y:%f,%f,%f,%f,%f,%f,%f,%f",x0,x1,x2,x3,y0,y1,y2,y3);

        while(t < 1.0)
        {
            double dt = len/(3*sqrt((xitem0 + xitem1*t + xitem2*t*t)*(xitem0 + xitem1*t + xitem2*t*t) +
                                    (yitem0 + yitem1*t + yitem2*t*t)*(yitem0 + yitem1*t + yitem2*t*t)));
            temp.pose.position.x = (1-t)*(1-t)*(1-t)*x0 + 3*t*(1-t)*(1-t)*x1 + 3*t*t*(1-t)*x2 + t*t*t*x3;
            temp.pose.position.y = (1-t)*(1-t)*(1-t)*y0 + 3*t*(1-t)*(1-t)*y1 + 3*t*t*(1-t)*y2 + t*t*t*y3;
            temp.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            path.push_back(temp);
            t += dt;
        }
        i += 3;
    }
    path.push_back(points[points.size()-1]);
    return true;
}

double MapServer::calculate_path_length(std::vector<geometry_msgs::PoseStamped>& path)
{
    double length = 0.0;
    if(0 == path.size())
    {
        return length;
    }

    double dist = 0.0;
    for(std::vector<geometry_msgs::PoseStamped>::iterator it=path.begin();it != path.end()-1; it++)
    {
        dist = sqrt((it->pose.position.x - (it+1)->pose.position.x)*(it->pose.position.x - (it+1)->pose.position.x)
                    +(it->pose.position.y - (it+1)->pose.position.y)*(it->pose.position.y - (it+1)->pose.position.y));
        length += dist;
    }
    return length;
}

bool MapServer::get_fixpath_info_by_map(std::string scene_name,std::string map_name,std::vector<path_t> &paths)
{
	sqlite3 *db;
	char *zErrMsg = 0;
	char *sql;
	//ROS_INFO("before open database");
	/* 创建或者打开数据库 */

	std::string file = this->config_map_path_ + scene_name +"/fixpathes/"+map_name+".db";
	if( sqlite3_open(file.c_str(), &db) )
	{
		ROS_ERROR("Can't open database: %s\n", sqlite3_errmsg(db));
		return false;
	}

	std::string s1 = INQUERY_OP;
	sql = (char *)s1.c_str();
	//sql = "SELECT * from PATH";

	ROS_INFO("before read path");
	std::vector<path_t>  p;
	if( sqlite3_exec(db, sql, database_callback, &p, &zErrMsg) != SQLITE_OK )
	{
		ROS_ERROR("SQL error: %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
		sqlite3_close(db);
		return false;
	}
	else
	{
		for(std::vector<path_t>::iterator it= p.begin();it != p.end();it++)
		{
			it->start = it->path[0];
			it->end = it->path[it->path.size()-1];
			it->scene_name = scene_name;
			it->map_name = map_name;
            it->length = calculate_path_length(it->path);//calculate path length
            it->points = it->path;
            if(1 == it->type)
            {
                bool success = create_bezier_curve_path(it->points);
                if(!success)
                {
                    ROS_ERROR("calculate bezier curve failed");
                    sqlite3_close(db);
                    return false;
                }
                it->length = calculate_path_length(it->points);
                ROS_INFO("bezier length:%f",it->length);
            }
		}
		paths = p;
	}
	sqlite3_close(db);
	ROS_INFO("after read path");
	return true;
}
bool MapServer::get_fixpath_info_by_id(int id, std::string map_name,std::string scene_name,Json::Value &path,const std::vector<point_t> &points)
{
    //step 2 calculate start and end point id;
    unsigned int i = 0;
    std::vector<path_t> map_paths;
    if(true == get_fixpath_info_by_map(scene_name,map_name,map_paths))
    {
        for(std::vector<path_t>::const_iterator path_it= map_paths.begin();path_it!= map_paths.end();++path_it)
        {
            if(id == path_it->id)
            {

                ROS_INFO("scene_name:%s,map_name:%s,path:id:%d",scene_name.c_str(),map_name.c_str(),path_it->id);
                path[i][ID] = path_it->id;
                path[i]["start_id"] = 0;
                path[i]["end_id"] = 0;
                path[i]["valid"] = 1;
                path[i]["type"] = path_it->type;
                path[i]["length"] = path_it->length;
                bool start_flag = false;
                bool end_flag = false;
                std::vector<point_t>::const_iterator start_it;
                std::vector<point_t>::const_iterator end_it;
                for(std::vector<point_t>::const_iterator point_it = points.begin();point_it != points.end();point_it++)
                {
                    if(check_same_point(point_it->pose,path_it->start) && (point_it->map_name == map_name) && (!start_flag))
                    {
                        start_it = point_it;
                        start_flag = true;
                    }
                    if(check_same_point(point_it->pose,path_it->end) && (point_it->map_name == map_name) && (!end_flag))
                    {
                        end_it = point_it;
                        end_flag = true;
                    }
                    if(start_flag && end_flag)
                    {
                        break;
                    }
                }
                if(start_flag && end_flag)
                {
                    path[i]["start_id"] = start_it->id;
                    path[i]["start_map"] = start_it->map_name;
                    path[i]["start_x"] = start_it->pose.x;
                    path[i]["start_y"] = start_it->pose.y;
                    path[i]["start_th"] = start_it->pose.theta;
                    path[i]["end_id"] = end_it->id;
                    path[i]["end_map"] = end_it->map_name;
                    path[i]["end_x"] = end_it->pose.x;
                    path[i]["end_y"] = end_it->pose.y;
                    path[i]["end_th"] = end_it->pose.theta;
                    path[i]["valid"] = 1;
                }
                else
                {
                    if(start_flag)
                    {
                        path[i]["start_id"] = start_it->id;
                        path[i]["start_map"] = start_it->map_name;
                        path[i]["start_x"] = start_it->pose.x;
                        path[i]["start_y"] = start_it->pose.y;
                        path[i]["start_th"] = start_it->pose.theta;
                    }
                    else
                    {
                        path[i]["start_id"] = 0;
                        path[i]["start_map"] = "null";
                        path[i]["start_x"] = path_it->start.pose.position.x;
                        path[i]["start_y"] = path_it->start.pose.position.y;
                        path[i]["start_th"] = tf::getYaw(path_it->start.pose.orientation);
                    }
                    if(end_flag)
                    {
                        path[i]["end_id"] = end_it->id;
                        path[i]["end_map"] = end_it->map_name;
                        path[i]["end_x"] = end_it->pose.x;
                        path[i]["end_y"] = end_it->pose.y;
                        path[i]["end_th"] = end_it->pose.theta;
                    }
                    else
                    {
                        path[i]["end_id"] = 0;
                        path[i]["end_map"] = "null";
                        path[i]["end_x"] = path_it->end.pose.position.x;
                        path[i]["end_y"] = path_it->end.pose.position.y;
                        path[i]["end_th"] = tf::getYaw(path_it->end.pose.orientation);
                    }
                    path[i]["valid"] = 0;
                }
                Json::Value points;
                Json::Value temp;
                if(0 == path_it->type)
                {
                    int j = 0;
                    temp["x"] = path[i]["start_x"];
                    temp["y"] = path[i]["start_y"];
                    temp["th"] = path[i]["start_th"];
                    points[j] =temp;
                    temp["x"] = path[i]["end_x"];
                    temp["y"] = path[i]["end_y"];
                    temp["th"] = path[i]["end_th"];
                    points[j+1] =temp;
                }
                else if(1 == path_it->type)
                {
                    for(int j = 0;j< path_it->path.size();j++)
                    {
                        temp["x"] = path_it->path[j].pose.position.x;
                        temp["y"] = path_it->path[j].pose.position.y;
                        temp["th"] = tf::getYaw(path_it->path[j].pose.orientation);
                        points[j] =temp;
                    }
                }
                else if(2 == path_it->type)
                {
                    for(int j = 0;j< path_it->points.size();j++)
                    {
                        temp["x"] = path_it->points[j].pose.position.x;
                        temp["y"] = path_it->points[j].pose.position.y;
                        temp["th"] = tf::getYaw(path_it->points[j].pose.orientation);
                        points[j] =temp;
                    }
                }
                path[i]["points"] = points;
                ++i;
            }
            else
            {
                continue;
            }
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool MapServer::get_fixpath_info_by_scene(const std::vector<std::string> &map_list,std::string scene_name,Json::Value &path,const std::vector<point_t> &points, bool query_all_points)
{
	//step 2 calculate start and end point id;
	unsigned int i = 0;
	for(std::vector<std::string>::const_iterator map_it = map_list.begin(); map_it!= map_list.end();++map_it)
	{
		std::vector<path_t> map_paths;
		std::string map_name = *map_it;
		if(true == get_fixpath_info_by_map(scene_name,map_name,map_paths))
		{
			for(std::vector<path_t>::const_iterator path_it= map_paths.begin();path_it!= map_paths.end();++path_it)
			{

				ROS_INFO("scene_name:%s,map_name:%s,path:id:%d",scene_name.c_str(),map_name.c_str(),path_it->id);
				path[i][ID] = path_it->id;
				path[i]["start_id"] = 0;
				path[i]["end_id"] = 0;
				path[i]["valid"] = 1;
				path[i]["type"] = path_it->type;
				path[i]["length"] = path_it->length;
				bool start_flag = false;
				bool end_flag = false;
				std::vector<point_t>::const_iterator start_it;
				std::vector<point_t>::const_iterator end_it;
				for(std::vector<point_t>::const_iterator point_it = points.begin();point_it != points.end();point_it++)
				{
					if(check_same_point(point_it->pose,path_it->start) && (point_it->map_name == map_name) && (!start_flag))
					{
						start_it = point_it;
						start_flag = true;
					}

					if(check_same_point(point_it->pose,path_it->end) && (point_it->map_name == map_name) && (!end_flag))
					{
						end_it = point_it;
						end_flag = true;
					}
					if(start_flag && end_flag)
					{
						break;
					}
				}
				if(start_flag && end_flag)
				{
					path[i]["start_id"] = start_it->id;
					path[i]["start_map"] = start_it->map_name;
					path[i]["start_x"] = start_it->pose.x;
					path[i]["start_y"] = start_it->pose.y;
					path[i]["start_th"] = start_it->pose.theta;
					path[i]["end_id"] = end_it->id;
					path[i]["end_map"] = end_it->map_name;
					path[i]["end_x"] = end_it->pose.x;
					path[i]["end_y"] = end_it->pose.y;
					path[i]["end_th"] = end_it->pose.theta;

					path[i]["valid"] = 1;
				}
				else
				{
					if(start_flag)
					{
						path[i]["start_id"] = start_it->id;
						path[i]["start_map"] = start_it->map_name;
						path[i]["start_x"] = start_it->pose.x;
						path[i]["start_y"] = start_it->pose.y;
						path[i]["start_th"] = start_it->pose.theta;
					}
					else
					{
						path[i]["start_id"] = 0;
						path[i]["start_map"] = "null";
						path[i]["start_x"] = path_it->start.pose.position.x;
						path[i]["start_y"] = path_it->start.pose.position.y;
						path[i]["start_th"] = tf::getYaw(path_it->start.pose.orientation);
					}
					if(end_flag)
					{
						path[i]["end_id"] = end_it->id;
						path[i]["end_map"] = end_it->map_name;
						path[i]["end_x"] = end_it->pose.x;
						path[i]["end_y"] = end_it->pose.y;
						path[i]["end_th"] = end_it->pose.theta;
					}
					else
					{
						path[i]["end_id"] = 0;
						path[i]["end_map"] = "null";
						path[i]["end_x"] = path_it->end.pose.position.x;
						path[i]["end_y"] = path_it->end.pose.position.y;
						path[i]["end_th"] = tf::getYaw(path_it->end.pose.orientation);
					}
					path[i]["valid"] = 0;
				}
				Json::Value points;
				Json::Value temp;
				if(!query_all_points)
				{
					if((0 == path_it->type) || (2 == path_it->type))
					{
						int j = 0;
						temp["x"] = path[i]["start_x"];
						temp["y"] = path[i]["start_y"];
						temp["th"] = path[i]["start_th"];
						points[j] =temp;
						temp["x"] = path[i]["end_x"];
						temp["y"] = path[i]["end_y"];
						temp["th"] = path[i]["end_th"];
						points[j+1] =temp;
					}
					else if(1 == path_it->type)
					{
						for(int j = 0;j< path_it->path.size();j++)
						{
							temp["x"] = path_it->path[j].pose.position.x;
							temp["y"] = path_it->path[j].pose.position.y;
							temp["th"] = tf::getYaw(path_it->path[j].pose.orientation);
							points[j] =temp;
						}
					}
					path[i]["points"] = points;
				}
				else
				{
					if(0 == path_it->type)
					{
						int j = 0;
						temp["x"] = path[i]["start_x"];
						temp["y"] = path[i]["start_y"];
						temp["th"] = path[i]["start_th"];
						points[j] =temp;
						temp["x"] = path[i]["end_x"];
						temp["y"] = path[i]["end_y"];
						temp["th"] = path[i]["end_th"];
						points[j+1] =temp;
					}
					else if(1 == path_it->type)
					{
						for(int j = 0;j< path_it->path.size();j++)
						{
							temp["x"] = path_it->path[j].pose.position.x;
							temp["y"] = path_it->path[j].pose.position.y;
							temp["th"] = tf::getYaw(path_it->path[j].pose.orientation);
							points[j] =temp;
						}
					}
					else if(2 == path_it->type)
					{
						for(int j = 0;j< path_it->points.size();j++)
						{
							temp["x"] = path_it->points[j].pose.position.x;
							temp["y"] = path_it->points[j].pose.position.y;
							temp["th"] = tf::getYaw(path_it->points[j].pose.orientation);
							points[j] =temp;
						}
					}
					path[i]["points"] = points;
				}
				++i;
			}
		}
		else
		{
			continue;
		}
	}
	return true;
}
bool MapServer::get_all_points(const std::vector<std::string> &map_list,std::string scene_name,std::vector<point_t> &points)
{
	std::string msg_info;
	for(std::vector<std::string>::const_iterator iter = map_list.begin();iter != map_list.end();iter++)
	{
		std::string path = (config_map_path_ + scene_name + "/point/" + *iter + ".yaml");
		ROS_ERROR("path:%s",path.c_str());
		//dataArray =  (*jptr)[DATA];
		std::vector<Point> vec;
		if(0 != loadPointYAML(path,vec,msg_info))
		{
			return false;
		}
		for(unsigned int i = 0; i < vec.size(); ++i)
		{

			point_t p;
			p.map_name = *iter;
			p.scene_name = scene_name;
			p.id = std::atoi((vec[i].nm).c_str());
			p.type = vec[i].type;
			p.pose.x = vec[i].x;
			p.pose.y = vec[i].y;
			p.pose.theta = vec[i].th;
			points.push_back(p);
		}

	}

	return true;

}
bool MapServer::save_path_database(path_t path)
{
	sqlite3 *db;
	char *zErrMsg = 0;
	char *sql;
	const char* data = "Callback function called";
	ROS_INFO("before open database");
	/* 创建或者打开数据库 */

	std::string file = this->config_map_path_ + path.scene_name+"/fixpathes/"+path.map_name+".db";
	if( sqlite3_open(file.c_str(), &db) )
	{
		ROS_ERROR("Can't open database: %s\n", sqlite3_errmsg(db));
		return false;
	}

	/* 创建一个表*/
	std::string s1 = CREATE_PATH_TABLE;
	sql = (char *)s1.c_str();
	if( sqlite3_exec(db, sql, database_callback, this, &zErrMsg) != SQLITE_OK )
	{
		ROS_ERROR("create path table failed: %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}

	int index = 0;
	/*表中插入数据*/
	for(std::vector<geometry_msgs::PoseStamped>::iterator it=path.path.begin(); it != path.path.end(); it++)
	{
		std::stringstream ss;
		ss<<INSERT_OP<<path.id<<", "<<path.type<<", "<<index<<", "<<it->pose.position.x<<", "<<it->pose.position.y<<", "<<tf::getYaw(it->pose.orientation)<<");";
		std::string op = ss.str();
		if( sqlite3_exec(db, op.c_str(), database_callback, this, &zErrMsg) != SQLITE_OK )
		{
			ROS_ERROR("insert database failed: %s\n", zErrMsg);
			sqlite3_free(zErrMsg);
			sqlite3_close(db);
			return false;
		}
		index++;
	}
	sqlite3_close(db);
	return true;
}
bool MapServer::check_same_point(const geometry_msgs::Pose2D p1,const geometry_msgs::PoseStamped p2)
{
	double dx = fabs(p1.x - p2.pose.position.x);
	double dy = fabs(p1.y - p2.pose.position.y);
	double dth = fabs(p1.theta - tf::getYaw(p2.pose.orientation));
	if((dx < 0.01) && (dy < 0.01) && (dth < 0.01))
	{
		return true;
	}
	return false;
}
bool MapServer::delete_fixpath(path_t path)
{
	sqlite3 *db;
	char *zErrMsg = 0;
	char *sql;

	//打开数据库
	std::string file = config_map_path_ + path.scene_name +"/fixpathes/" +path.map_name+".db";
	if( sqlite3_open(file.c_str(), &db) )
	{
		ROS_ERROR("Can't open database: %s\n", sqlite3_errmsg(db));
		return false;
	}

	std::stringstream id;
	id<<path.id;
	std::string s1 = DELETE_OP + id.str();
	sql = (char *)s1.c_str();

	if( sqlite3_exec(db, sql, 0, 0, &zErrMsg) != SQLITE_OK )
	{
		ROS_ERROR("SQL error: %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
		sqlite3_close(db);
		return false;
	}
	sqlite3_close(db);
	return true;
}


double MapServer::line_dist(double x0,double y0,double x1,double y1)
{
	double dist = 0.0;

	dist = sqrt(pow(x0 - x1,2)+pow(y0 - y1,2));
	return dist;
}
double MapServer::adjust_th(double th)
{
	if(th >= M_PI)
	{
		while(th >= M_PI)
		{
			th -= 2*M_PI;
		}
	}
	else if(th < -M_PI)
	{
		while(th < -M_PI)
		{
			th += 2*M_PI;
		}
	}
	return th;
}
bool MapServer::get_fixpath_service(map_server::GetFixPath::Request &req,map_server::GetFixPath::Response &res)
{
    ROS_INFO("receive a new request to get a fixpath");
    //get fix path through id.
    std::vector<path_t> paths;
    std::string scene_name = req.path_scene_name;
    std::string map_name = req.path_map_name;//TODO map name
    unsigned int id = req.path_id;
    bool flag = get_fixpath_info_by_map(scene_name,map_name,paths);
    if(!flag)
    {
        res.path_found = 1;
        res.error_message="get this scene fixpaths failed";
        return false;
    }
    for(std::vector<path_t>::iterator it=paths.begin();it!=paths.end();it++)
    {
        if(it->id == req.path_id)
        {
            res.path_found = 0;
            res.error_message="get path success";
            res.path = it->points;
            return true;
        }
    }
    res.path_found = 2;
    res.error_message="get path failed";
    return false;
}
#endif /* MAP_SERVER_INCLUDE_MAP_SERVER_FIXPATH_OPERATIONS_HPP_ */
