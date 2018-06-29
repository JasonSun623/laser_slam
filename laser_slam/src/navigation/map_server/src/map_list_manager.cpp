/*
 * map_manager.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: tomzhang
 */


#include "map_server/map_list_manager.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
namespace map_server{

MapListManager::MapListManager(const std::string &config_path):config_map_path_(config_path)
{

}

MapListManager::~MapListManager(){

}


int MapListManager::get_map_list(std::vector<scene> &vec)
{
	DIR *dir = NULL;
	dir = opendir(config_map_path_.c_str());
	if(NULL == dir)
	{
		ROS_INFO("Opendir %s failed!", config_map_path_.c_str());
		return -1;
	}
	//change dictionary to the scene folder
	chdir(config_map_path_.c_str());
	dirent *ptr = NULL;
	while(NULL!=(ptr=readdir(dir)))
	{// read items one by one
		if(DT_DIR==ptr->d_type)
		{ // if the item is a folder
			if((0!=strcmp(ptr->d_name,"."))&&(0!=strcmp(ptr->d_name,"..")))
			{
				scene sce;
				std::string scene_path = ptr->d_name;
				sce.scene_name = scene_path;
				//ROS_ERROR("1");
				traverse_folder(sce);
				vec.push_back(sce);
			}
		}
	}
	chdir("..");
	closedir(dir);
	return 0;
}

void MapListManager::traverse_folder(scene &sce)
{
	//ROS_ERROR("2");
	std::string path = config_map_path_+sce.scene_name+"/map";
	DIR *dir = NULL;
	//open the folder
	dir = opendir(path.c_str());
	if(NULL == dir)
	{
		ROS_INFO("Opendir %s failed!", sce.scene_name.c_str());
		return;
	}
	chdir(path.c_str());
	//ROS_ERROR("%s",path.c_str());
	dirent *ptr = NULL;
	while(NULL!=(ptr=readdir(dir)))
	{// read items one by one
		if(DT_REG==ptr->d_type)
		{ // if the item is a regular file
			if((0 != strcmp(ptr->d_name,"."))&&(0 != strcmp(ptr->d_name,"..")))
			{
				std::string map_name = ptr->d_name;
				//ROS_ERROR("%s",map_name.c_str());
				std::string ext = ".yaml";
				if(map_name.length()>ext.length())
				{
					std::string ext_name = map_name.substr(map_name.length()-ext.length(),ext.length());
					if(0==strcmp(ext_name.c_str(),ext.c_str())){
						std::string fname = map_name.substr(0,map_name.length()-ext.length());
						sce.map_list.push_back(fname);
					}
				}


			}
		}
	}
	chdir("..");
	chdir("..");
	closedir(dir);

}

int MapListManager::rename_map(const std::string &old_map_name,const std::string &new_map_name )
{
	std::string old_file = config_map_path_ + old_map_name;
	std::string new_file = config_map_path_ + new_map_name;
	return rename(old_file.c_str(), new_file.c_str());
}
int MapListManager::rename_scene(const std::string &old_scene_name,const std::string &new_scene_name)
{
	std::string old_file = config_map_path_ + old_scene_name;
	std::string new_file = config_map_path_ + new_scene_name;
	return rename(old_file.c_str(),new_file.c_str());
}
int MapListManager::delete_map(const std::string &scene_name, const std::string &map_name)
{
	std::string map_yaml = config_map_path_ + scene_name + "/map/" + map_name + ".yaml";
	std::string label_yaml = config_map_path_ + scene_name + "/label/" + map_name + ".yaml";
	std::string map_pgm = config_map_path_ + scene_name + "/map/" + map_name + ".pgm";
	std::string map_png = config_map_path_ + scene_name + "/map/" + map_name + ".png";
	std::string point_yaml = config_map_path_ + scene_name + "/point/" + map_name + ".yaml";
	std::string virtual_wall_yaml = config_map_path_ + scene_name + "/virtual_wall/" + map_name + ".yaml";
	std::string fixpathes_db = config_map_path_ + scene_name + "/fixpathes/" + map_name + ".db";
	std::string regions_db = config_map_path_ + scene_name + "/regions/" + map_name + ".db";

	remove(label_yaml.c_str());
	remove(map_yaml.c_str());
	remove(map_pgm.c_str());
	remove(map_png.c_str());
	remove(point_yaml.c_str());
	remove(virtual_wall_yaml.c_str());
	remove(fixpathes_db.c_str());
	remove(regions_db.c_str());
	return 0;
}
int MapListManager::delete_scene(const std::string &name)
{

	//std::string cmd = "rm -rf ";
	//cmd += scene_name;
	//system(cmd.c_str());

	DIR *dir = NULL;
	dir = opendir(name.c_str());
	if(dir ==NULL){
		ROS_INFO("Opendir %s failed!", name.c_str());
		return -1;
	}

	chdir(name.c_str());
	dirent *ptr = NULL;
	//ROS_ERROR("1");
	while((ptr=readdir(dir))!=NULL){// read items one by one
		if(ptr->d_type==DT_DIR)
		{ // if the item is a folder
			if((0!=strcmp(ptr->d_name,"."))&&(0!=strcmp(ptr->d_name,".."))){
				//ROS_ERROR("%s,ptr->d_type:%d",ptr->d_name, ptr->d_type);
				std::string foldername;
				foldername = ptr->d_name;
				delete_scene(foldername);
				remove(foldername.c_str());
			}
		}
		else if(ptr->d_type==DT_REG)
		{ // if the item is a regular file
			//ROS_ERROR("%s,ptr->d_type:%d",ptr->d_name, ptr->d_type);
			if((0!=strcmp(ptr->d_name,"."))&&(0!=strcmp(ptr->d_name,".."))){
				std::string filename = ptr->d_name;
				remove(filename.c_str());
			}
		}
	}
	chdir("..");
	remove(name.c_str());
	closedir(dir);
	return 0;
}  
}
