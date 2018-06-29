/*
 * map_manager.h
 *
 *  Created on: Feb 9, 2017
 *      Author: tomzhang
 */

#ifndef INCLUDE_MAP_SERVER_MAP_LIST_MANAGER_H_
#define INCLUDE_MAP_SERVER_MAP_LIST_MANAGER_H_



#include <stdio.h>
#include <string>
#include <vector>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>
namespace map_server {

struct scene{
	std::string scene_name;
	std::vector<std::string> map_list;
};


class MapListManager{


public:
	explicit MapListManager(const std::string &config_path);
	~MapListManager();
	int rename_map(const std::string &old_map_name,const std::string &new_map_name );
	int rename_scene(const std::string &old_scene_name,const std::string &new_scene_name);
	//int rename_scene(const std::string scene_name);
	int delete_map(const std::string &scene_name, const std::string &map_name);
	int delete_scene(const std::string &name);
	int get_map_list(std::vector<scene> &vec);
	void traverse_folder(scene &sce);
	void setMapPath(const std::string &config_path);


private:
	const std::string &config_map_path_;


};


};

#endif /* INCLUDE_MAP_SERVER_MAP_LIST_MANAGER_H_ */
