/*
 * map_server_functions.hpp
 *
 *  Created on: Apr 5, 2017
 *      Author: tomzhang
 */

#ifndef MAP_SERVER_INCLUDE_MAP_SERVER_MAP_SERVER_FUNCTIONS_HPP_
#define MAP_SERVER_INCLUDE_MAP_SERVER_MAP_SERVER_FUNCTIONS_HPP_


#include "map_server/map_server_mrobot.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include "convert/PGM2PNG.h"
#include "convert/PGM2PNGRequest.h"
#include "convert/PGM2PNGResponse.h"
#include <unistd.h>
#include "map_server/map_list_manager.h"
using namespace map_server;
void MapServer::initFuncsMap()
{
	function_map_ptr_ = FuncMapPtr(new FuncMap());

	(*function_map_ptr_)[CMD_MAP_LIST_GET]    =boost::bind(&MapServer::map_list_get,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_MAP_RENAME]      =boost::bind(&MapServer::map_rename,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_MAP_CURRENT_GET] =boost::bind(&MapServer::map_current_get,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_MAP_SCENE_RENAME]=boost::bind(&MapServer::map_scene_rename,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_MAP_DELETE]      =boost::bind(&MapServer::map_delete,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_SCENE_DELETE]    =boost::bind(&MapServer::map_scene_delete,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_MAP_SAVE]        =boost::bind(&MapServer::map_save,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_MAP_LOAD]        =boost::bind(&MapServer::map_load,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_MAP_EDIT]        =boost::bind(&MapServer::map_edit,this,_1,_2,_3,_4);
	(*function_map_ptr_)[MAP_EDIT_COMPLETE]   =boost::bind(&MapServer::map_edit_complete,this,_1,_2,_3,_4);

	(*function_map_ptr_)[CMD_POINT_ADD]       =boost::bind(&MapServer::point_add,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_POINT_DELETE]    =boost::bind(&MapServer::point_delete,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_POINT_QUERY]     =boost::bind(&MapServer::point_query,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_POINT_EDIT]      =boost::bind(&MapServer::point_edit,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_POINT_LOAD]      =boost::bind(&MapServer::point_load,this,_1,_2,_3,_4);

    (*function_map_ptr_)[CMD_LABEL_SAVE]      =boost::bind(&MapServer::label_save,this,_1,_2,_3,_4);
	(*function_map_ptr_)[CMD_LABEL_DELETE]      =boost::bind(&MapServer::label_delete,this,_1,_2,_3,_4);

    (*function_map_ptr_)[CMD_VIRTUAL_WALL_LOAD]  =boost::bind(&MapServer::virtual_wall_load,this,_1,_2,_3,_4);
    (*function_map_ptr_)[CMD_VIRTUAL_WALL_ADD]  =boost::bind(&MapServer::virtual_wall_add,this,_1,_2,_3,_4);
    (*function_map_ptr_)[CMD_VIRTUAL_WALL_DELETE]  =boost::bind(&MapServer::virtual_wall_delete,this,_1,_2,_3,_4);
    function_region_ptr_ = FuncMapPtr(new FuncMap());
    (*function_region_ptr_)[CMD_REGIONS_SCENE_GET]  =boost::bind(&MapServer::regions_scene_get,this,_1,_2,_3,_4);
    (*function_region_ptr_)[CMD_REGIONS_MAP_GET]  =boost::bind(&MapServer::regions_map_get,this,_1,_2,_3,_4);
    (*function_region_ptr_)[CMD_REGION_CREATE]  =boost::bind(&MapServer::region_create,this,_1,_2,_3,_4);
    (*function_region_ptr_)[CMD_REGION_REMOVE]  =boost::bind(&MapServer::region_remove,this,_1,_2,_3,_4);
    (*function_region_ptr_)[CMD_REGION_PARAMS_GET]  =boost::bind(&MapServer::region_params_get,this,_1,_2,_3,_4);
    (*function_region_ptr_)[CMD_REGION_MODIFY]  =boost::bind(&MapServer::region_modify,this,_1,_2,_3,_4);

    (*function_map_ptr_)[FIXPATH_PARAM_ADD]  =boost::bind(&MapServer::fixpath_param_add,this,_1,_2,_3,_4);
    (*function_map_ptr_)[FIXPATH_QUERY]  =boost::bind(&MapServer::fixpath_query,this,_1,_2,_3,_4);
    (*function_map_ptr_)[FIXPATH_MAP_QUERY]  =boost::bind(&MapServer::fixpath_map_query,this,_1,_2,_3,_4);
    (*function_map_ptr_)[FIXPATH_PATH_QUERY]  =boost::bind(&MapServer::fixpath_path_query,this,_1,_2,_3,_4);
    (*function_map_ptr_)[FIXPATH_FILE_QUERY]  =boost::bind(&MapServer::fixpath_file_query,this,_1,_2,_3,_4);
    (*function_map_ptr_)[FIXPATH_DELETE]  =boost::bind(&MapServer::fixpath_delete,this,_1,_2,_3,_4);
    (*function_map_ptr_)[FIXPATH_EDIT]  =boost::bind(&MapServer::fixpath_edit,this,_1,_2,_3,_4);
    (*function_map_ptr_)[RECORD_PATH_START]  =boost::bind(&MapServer::fixpath_record_start,this,_1,_2,_3,_4);
    (*function_map_ptr_)[RECORD_PATH_END]  =boost::bind(&MapServer::fixpath_record_finish,this,_1,_2,_3,_4);

}

// get the scene and map list, create json
void MapServer::map_list_get(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    std::vector<map_server::scene> scene_list;
    if(0!= this->map_list_manager_->get_map_list(scene_list))
    {
        err = 2;
        msg_info = "Get list failed!";
        return;
    }
    else
    {
        if(scene_list.empty())
        {
            msg_info = "No Maps";
        }
        else
        {
            for (std::vector<map_server::scene>::iterator iter =
                    scene_list.begin(); iter != scene_list.end(); ++iter)
            {
                Json::Value scene;
                Json::Value arrayObj;

                scene["name"] = iter->scene_name;

                int i = 0;
                if (!iter->map_list.empty())
                {
                    for (std::vector<std::string>::iterator it =
                            iter->map_list.begin(); it != iter->map_list.end();
                            it++)
                    {
                        Json::Value map;
                        map["id"] = i++;
                        map["name"] = (*it);
                        arrayObj.append(map);
                    }

                }
                scene[MAP_NAMES] = arrayObj;
                dataArray.append(scene);
                msg_info = "Succeed!";
            }


        }

    }
}
void MapServer::map_current_get(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    if(!isMapLoad())
    {
        err = 2;
        msg_info = "No map loaded currently!";
        return;
    }
    dataArray[SCENE_NAME] = this->current_scene_;
    dataArray[MAP_NAME] = this->current_map_;
    msg_info.append("Succeed!");
    err = 0;

}
void MapServer::map_rename(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    std::string scene_name_str, old_name_str, new_name_str;
    if (0 != (err =mapRenameDataExtract(jptr,scene_name_str, old_name_str, new_name_str,msg_info)))
    {
        return;
    }
    dataArray =  (*jptr)[DATA];
    if(this->current_map_ == old_name_str && this->current_scene_ == scene_name_str)
    {
        err = 2;
        msg_info.append("Rename Failed, you can not rename the current map!");
        return;
    }
    if(this->current_map_ == new_name_str && this->current_scene_ == scene_name_str)
    {
        err = 6;
        msg_info.append("Rename Failed, you can not cover the current map!");
        return;
    }
    if (0 != (err = checkString(new_name_str,msg_info)))
    {
        msg_info.append(" Map rename is Failed!");
        return;
    }
    err = renameMap(scene_name_str, old_name_str, new_name_str,msg_info);
    if(0==err&&scene_name_str == this->current_scene_)
    {
        std::map<std::string, boost::shared_ptr<nav_msgs::GetMap::Response> >::iterator it;
        if ((it = maps_.find(old_name_str)) != maps_.end())
        {
            maps_.erase(it);
        }
        if ((it = maps_.find(new_name_str)) != maps_.end())
        {
            maps_.erase(it);
        }
    }
}
void MapServer::map_scene_rename(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    std::string old_scene_str, new_scene_str;
    if(0!=(err=this->mapSceneRenameDataExtract(jptr,old_scene_str,new_scene_str,msg_info))) return;
    dataArray =  (*jptr)[DATA];
    if (0 != (err = checkString(new_scene_str,msg_info)))
    {
        msg_info.append(" Scene rename is failed!");
        return;
    }
    if (0 != map_list_manager_->rename_scene(old_scene_str,new_scene_str))
    {
        err = 2;
        msg_info.append("System error, scene rename failed!");
        return;
    }
    if(old_scene_str == this->current_scene_)
    {
        this->current_scene_ = new_scene_str;
    }
    msg_info.append("Scene rename succeed!");
}
void MapServer::map_delete (Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    std::string scene_name_str,map_name_str;
    if(0!=(err=this->mapDeleteDataExtract(jptr,scene_name_str,map_name_str,msg_info))) return;
    dataArray =  (*jptr)[DATA];
    if(scene_name_str == this->current_scene_&& map_name_str == this->current_map_)
    {
        err = 2;
        msg_info.append("The map is using currently, please use another map name!");
        return;

    }
    err = map_list_manager_->delete_map(scene_name_str, map_name_str);
    if(err != 0)
    {
        msg_info.append("Delete Map Failed!");
        return;
    }
    msg_info.append("Delete Map Succeed!");

}
void MapServer::map_scene_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{

    std::string scene_name_str;
    if(0!=(err=this->mapSceneDeleteDataExtract(jptr,scene_name_str,msg_info))) return;
    dataArray =  (*jptr)[DATA];
    if(scene_name_str == this->current_scene_)
    {
        err = 2;
        msg_info.append("The scene is loaded, you can not delete it!");
        return;
    }
    if (0!=(err = map_list_manager_->delete_scene(config_map_path_ + scene_name_str)))
    {
        err = 3;
        msg_info.append("System error, delete failed!");
        return;
    }
    msg_info.append("Succeed!");

}
void MapServer::map_save(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{

    std::string scene_name_str, map_name_str;

    if(0!=(err=this->mapSaveDataExtract(jptr,scene_name_str,map_name_str,msg_info))) return;
    dataArray =  (*jptr)[DATA];
    if (0 != (err = checkString(scene_name_str,msg_info)))
    {
        msg_info.append(" please use valid characters or length for scene name!");
        return;
    }
    if (0 != (err = checkString(map_name_str,msg_info)))
    {
        err+=3;
        msg_info.append(" please use valid characters or length for map name!");
        return;
    }

    if(this->current_scene_ == scene_name_str)
    {
        if(this->current_map_ == map_name_str)
        {
            err = 9;
            msg_info = map_name_str+" is showing currently, please use another name!";
            return ;
        }
        else
        {
            std::map<std::string, boost::shared_ptr<nav_msgs::GetMap::Response> >::iterator it;
            if((it=this->maps_.find(map_name_str))!=maps_.end())
            {
                this->maps_.erase(it);
            }
        }
    }
    // we check whether the scene folder exist in system, if not, create it.
    if(0!=(err = folder_check(scene_name_str,msg_info))) return;

    //allow callback function to store map data
    //start_save_map_ = true;
    //wait until map data from other node come in
    boost::mutex::scoped_lock lock(save_map_mtx_);
    if(save_map_ptr_!=NULL)
    {
        //save map and free memory
        err = save_map_to_file (save_map_ptr_,scene_name_str, map_name_str,msg_info);
        save_map_ptr_.reset();
    }
    else
    {
        err = 2;
        msg_info="No map data received!";
    }
    convert::PGM2PNG srv;
    srv.request.scene_name = config_map_path_ + scene_name_str;
    srv.request.map_name = map_name_str;
    convert_client_.call(srv);
}
void MapServer::map_load(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    std::string scene_name_str, map_name_str;

    if(0!=(err=this->mapLoadDataExtract(jptr,scene_name_str,map_name_str,msg_info))) return;
    if(0!=(err=folder_check(scene_name_str, msg_info)))return;
    int k = labelVectorInit(scene_name_str,map_name_str);
    ROS_INFO("map_load;initial label vector %d",k);
    dataArray[SCENE_NAME] = scene_name_str;
    dataArray[MAP_NAME] =  map_name_str;

    // if the scene of the loading map  is not current , we should clear the map container and then load all maps from this scene.
    if (scene_name_str != current_scene_)
    {
        current_scene_ = scene_name_str;
        if (!maps_.empty())
        {
            maps_.clear();
        }
        loadAllMaps(scene_name_str);
    }
    std::map<std::string, boost::shared_ptr<nav_msgs::GetMap::Response> >::iterator it;
    // find the map in container and then publish
    if ((it = maps_.find(map_name_str)) != maps_.end())
    {

        map_resp_ = it->second;
        current_map_ = map_name_str;
    }
    else
    {// if we could not find it in the container, first check whether we did not load it in the container, if not, return failed.
        boost::shared_ptr<nav_msgs::GetMap::Response> rep_ptr(new nav_msgs::GetMap::Response());
        if(0!= readPGM(config_map_path_ + scene_name_str + "/map/" + map_name_str, rep_ptr))
        {
            err =2;
            msg_info = "Load map "+map_name_str+" failed!";
            return;
        }
        // if we loaded it, add it to the container
        maps_[map_name_str] = rep_ptr;
        map_resp_ = rep_ptr;
        current_map_ = map_name_str;

    }

    {
        Json::Value mapsArray;
        for (std::map<std::string, boost::shared_ptr<nav_msgs::GetMap::Response> >::iterator it =maps_.begin(); it != maps_.end(); ++it)
        {
            mapsArray.append(it->first);
        }

        dataArray[SCENE_NAME] = scene_name_str;

        dataArray["mapNames"] = mapsArray;
        dataArray[MAP_NAME] =  current_map_;

        Json::Value json_map_list;
        Json::FastWriter writer;
        json_map_list["current_map_name"] = map_name_str;
        json_map_list["map_list"] = dataArray;

        std_msgs::String output_map_list;
        output_map_list.data = writer.write(json_map_list);
        map_list_pub_.publish(output_map_list);
        map_resp_->map.header.stamp = ros::Time::now();
        map_pub_.publish(map_resp_->map);
        metadata_pub.publish(map_resp_->map.info);
        pub_map_filter(map_resp_->map);
        msg_info = "Load Map Succeed!";
    }
    this->rmgr_->openRegionsBD(scene_name_str, map_name_str);
    this->setMapLoad(true);
}

void MapServer::pub_map_filter(const nav_msgs::OccupancyGrid map)
{
    map_filter_.header = map.header;
    map_filter_.info = map.info;
    int filter_obstacle_size = 0;
    int filter_free_size = 0;
    int i,tmp;
    for(i=0; i < map.data.size(); i++)
    {
        tmp = map.data[i];
        if(100 == tmp)
        {
            filter_obstacle_size++;
        }
        else if(0 == tmp)
        {
            filter_free_size++;
        }
    }
    map_filter_.data_obstacle.resize(filter_obstacle_size);
    map_filter_.data_free.resize(filter_free_size);
    int filter_obstacle_count = 0;
    int filter_free_count = 0;
    for(i=0; i < map.data.size(); i++)
    {
        tmp = map.data[i];
        if(100 == tmp)
        {
            map_filter_.data_obstacle[filter_obstacle_count] = i;
            filter_obstacle_count++;
        }
        else if(0 == tmp)
        {
            map_filter_.data_free[filter_free_count] = i;
            filter_free_count++;
        }
    }
    map_filter_pub_.publish(map_filter_);
}

void MapServer::map_edit(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    std::string scene_name_str, map_name_str;
    if(0!=(err=this->mapSaveDataExtract(jptr,scene_name_str, map_name_str, msg_info))) return;
    dataArray= (*jptr)[DATA];

    if(true == start_edit_map_)
    {
        err = 2;
        msg_info = "Map Server is receiving data from another source!";
        return;
    }
    else
    {
        start_edit_map_ = true;

        msg_info.append("Ready for receiving data!");
        Jptr j = boost::make_shared<Json::Value>(Json::Value());
        (*j) = (*jptr);
        (*j)[PUB_NAME] = MAP_EDIT_COMPLETE;
        {
              boost::mutex::scoped_lock lock_(task_mtx_);
              cmd_processor_->pushTask(boost::bind(&MapServer::taskRunner,this,(*function_map_ptr_)[MAP_EDIT_COMPLETE],jptr,MAP_EDIT_COMPLETE,app_pub_));
        }
        cond_pull_.notify_one();
    }
}

void MapServer::map_edit_complete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    std::string scene_name_str, map_name_str;
    if(0!=(err=this->mapSaveDataExtract(jptr,scene_name_str, map_name_str, msg_info))) return;
    dataArray= (*jptr)[DATA];
    while(edit_map_ptr_==NULL)
    {
        if(edit_map_cond_.timed_wait(edit_map_mtx_,boost::get_system_time() + boost::posix_time::seconds(8)))
        {
            break;
        }
        else
        {

            err = 2;
            msg_info="Time Out, no map data received!";
            start_edit_map_ = false;
            return;
        }
    }
    //save map and free memory
    err = save_map_to_file (edit_map_ptr_,scene_name_str, map_name_str,msg_info);
    if(err == 0)
    {
        msg_info = "Map edit Succeed! Please reload map to take effect!";
        if(scene_name_str == this->current_scene_)
        {
            std::map<std::string, boost::shared_ptr<nav_msgs::GetMap::Response> >::iterator it;
            if ((it = maps_.find(map_name_str)) != maps_.end())
            {
                maps_.erase(it);
            }
        }

    }

    edit_map_ptr_.reset();
}

void MapServer::point_add(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	std::string scene_name_str, map_name_str;
	Point p;
	if(0!=(err=pointAddDataExtract(jptr,scene_name_str,map_name_str,p,msg_info))) return;
	std::string path = (config_map_path_ + scene_name_str + "/point/" + map_name_str + ".yaml");
	dataArray =  (*jptr)[DATA];
	if(0!=(err=writePointYAML(path,p,msg_info)))return ;
	dataArray[POINT_NAME] = p.nm;
}
void MapServer::point_load(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{

	std::string scene_name_str, map_name_str;
	if(0!=(err=pointLoadDataExtract(jptr,scene_name_str,map_name_str,msg_info)))return;
	std::vector<Point> vec;
    if(0 != access((config_map_path_ + scene_name_str+"/point").c_str(),0))
    {
        ROS_INFO("No point directory");
        mkdir((config_map_path_ + scene_name_str+"/point").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
	std::string path = (config_map_path_ + scene_name_str + "/point/" + map_name_str + ".yaml");
	dataArray =  (*jptr)[DATA];
	if(0!=(err=loadPointYAML(path,vec,msg_info)))return;

	Json::Value pointdata;
	for(unsigned int i = 0; i < vec.size(); ++i)
	{
		pointdata[i][POINT_NAME] = vec[i].nm;
		pointdata[i][POINT_ALIAS] = vec[i].alias;
		pointdata[i][POINT_TYPE] = vec[i].type;
		pointdata[i][X] = vec[i].x;
		pointdata[i][Y] = vec[i].y;
		pointdata[i][TH]= vec[i].th;
	}
	dataArray[POINTS] = pointdata;
}
void MapServer::point_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	std::string scene_name_str, map_name_str;
	Point p;
	if(0!=(err=pointDeleteExtract(jptr,scene_name_str,map_name_str,p,msg_info))) return;
	std::string path = (config_map_path_ + scene_name_str + "/point/" + map_name_str + ".yaml");
	dataArray =  (*jptr)[DATA];
	if(0!=(err=deletePointYAML(path,p,msg_info)))return;
}
void MapServer::point_query(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	std::string scene_name_str, map_name_str;
	Point p;
	if(0!=(err=this->pointQueryDataExtract(jptr,scene_name_str,map_name_str,p,msg_info))) return;
	std::string path = (config_map_path_ + scene_name_str + "/point/" + map_name_str + ".yaml");
	dataArray =  (*jptr)[DATA];
	if(0!=(err=queryPointYAML(path,p,msg_info)))return;

	dataArray[POINT_NAME] = p.nm;
	dataArray[POINT_ALIAS] = p.alias;
	dataArray[POINT_TYPE] = p.type;
	dataArray[X] = p.x;
	dataArray[Y] = p.y;
	dataArray[TH] = p.th;

}
void MapServer::point_edit(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
    std::string scene_name_str, map_name_str,point_name_str;
    Point p;
    if(0!=(err=pointEditDataExtract(jptr,scene_name_str,map_name_str,point_name_str,p,msg_info))) return;
    std::string path = (config_map_path_ + scene_name_str + "/point/" + map_name_str + ".yaml");
    dataArray =  (*jptr)[DATA];
    if(0!=(err=editPointYAML(path,point_name_str,p,msg_info)))return;
}

int MapServer::checkString(const std::string str, std::string &msg_info)
{
    // check string length
    if (str.length() < 1 || str.length() > 200)
    {
        msg_info = "Characters length invalid!";
        return 3;
    }
    if(str[0]=='+'||str[0]=='-'||str[0]=='.'||str[0]=='*'||str[0]=='$')
    {
        msg_info = "Characters invalid!";
        return 4;
    }
    for (size_t i = 0; i < str.length(); i++)
    {

        if(str[i] == '\\')
        {
            msg_info ="\\ is not allowed!";
            return 5;
        }
    }
    return 0;
}



int MapServer::renameMap(const std::string scene, const std::string oldname,
                         const std::string newname,  std::string & msg_info)
{
    std::string old_map =scene + "/map/" + oldname + ".pgm";
    std::string old_png_map =scene + "/map/" + oldname + ".png";
    std::string old_yaml = scene +"/map/" +oldname + ".yaml";
    std::string old_point_name =scene + "/point/" + oldname +".yaml";
    std::string old_wall_name =scene + "/virtual_wall/" + oldname +".yaml";
    std::string old_label_name =scene + "/label/" + oldname +".yaml";
    std::string old_fixpath_name =scene + "/fixpathes/" + oldname +".db";
    std::string old_regions_name =scene + "/regions/" + oldname +".db";


    std::string new_map =scene + "/map/" +newname + ".pgm";
    std::string new_png_map =scene + "/map/" +newname + ".png";
    std::string new_yaml =scene + "/map/" +newname + ".yaml";
    std::string new_point_name =scene + "/point/" + newname +".yaml";
    std::string new_wall_name =scene + "/virtual_wall/" + newname +".yaml";
    std::string new_label =scene + "/label/" +newname + ".yaml";
    std::string new_fixpath_name =scene + "/fixpathes/" + newname +".db";
    std::string new_regions_name =scene + "/regions/" + newname +".db";

    int overwrite = checkMapName(scene,newname);

    int err = map_list_manager_->rename_map(old_map, new_map);
    if (err != 0)
    {
        ROS_ERROR("Map_server could not find %s.", (old_map).c_str());
        msg_info = "Map data not found, rename failed!";
        return 6;
    }
    err = map_list_manager_->rename_map(old_yaml,new_yaml);
    if (err != 0)
    {
        ROS_ERROR("Map_server could not find %s.", (old_yaml).c_str());
        msg_info = "Map file not found, rename failed!";
        return 7;
    }
    std::ifstream fin((new_yaml).c_str());
    if (fin.fail())
    {
        ROS_ERROR("Map_server could not open %s.", (new_yaml).c_str());
        fin.close();
        msg_info = "Map file cannot open, rename failed";
        return 9;
    }

    YAML::Node doc = YAML::Load(fin);
    doc["image"] = (newname + ".pgm");
    std::ofstream fout((new_yaml).c_str());
    fin.close();
    if (fout.fail())
    {
        ROS_ERROR("Map_server could not open %s.", (new_yaml).c_str());
        fout.close();
        msg_info = "System error, rename failed";
        return 10;
    }
    fout << doc;
    fout.close();

    err = map_list_manager_->rename_map(old_label_name, new_label);
    if (err != 0)
    {
        ROS_ERROR("Map_server could not find %s.", (old_label_name).c_str());
        //msg_info = "Label not found, rename failed!";
        //return 6;
    }
    err = map_list_manager_->rename_map(old_png_map,new_png_map);
    if (err != 0)
    {
        ROS_ERROR("Map_server could not find %s.", (old_png_map).c_str());
        //msg_info = "Map PNG file not found, rename failed!";
       // return 12;
    }
    err = map_list_manager_->rename_map(old_point_name,new_point_name);
    if (err != 0)
    {
        ROS_ERROR("Map_server could not find %s.", (old_point_name).c_str());
        //msg_info = "Point file not found";
       // return 8;
    }
    err = map_list_manager_->rename_map(old_wall_name,new_wall_name);
    if (err != 0)
    {
        ROS_ERROR("Map_server could not find %s.", (old_wall_name).c_str());
        //msg_info = "Wall file not found!";
        //  return 11;
    }
    err = map_list_manager_->rename_map(old_fixpath_name,new_fixpath_name);
    if (err != 0)
    {
        ROS_ERROR("Map_server could not find %s.", (old_fixpath_name).c_str());
        //msg_info = "Wall file not found!";
        //  return 11;
    }
    err = map_list_manager_->rename_map(old_regions_name,new_regions_name);
    if (err != 0)
    {
        ROS_ERROR("Map_server could not find %s.", (old_regions_name).c_str());
        //msg_info = "Wall file not found!";
        //  return 11;
    }

    if(0 != overwrite)
        msg_info= "Rename Succeed";
    else
        msg_info = "This map name has existed! The new map will cover the old map!";
    return 0;
}

int MapServer::checkMapName(const std::string &scene, const std::string &map)
{
    map_server::scene sce;
    sce.scene_name = scene;
    map_list_manager_->traverse_folder(sce);
    if(find(sce.map_list.begin(),sce.map_list.end(),map)!=sce.map_list.end())
    {
        return 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           ;
    }
    return -1;
}
int MapServer::folder_check(const std::string& save_scene_name, std::string &msg_info)
{

        if (0 != access((config_map_path_ + save_scene_name).c_str(), F_OK))
        {
		    if (-1 == mkdir((config_map_path_ + save_scene_name).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            msg_info = "System error, can not create scene folder! ";
            return 10;

        }
        }

		if (-1 == access((config_map_path_ + save_scene_name + "/map").c_str(), F_OK))
    	{
        if (-1 == mkdir((config_map_path_ + save_scene_name + "/map").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            msg_info = "System error, can not create map folder!";
            return 11;
        }
		}
		if (-1 == access((config_map_path_ + save_scene_name + "/point").c_str(), F_OK))
    	{
        if (-1 == mkdir((config_map_path_ + save_scene_name + "/point").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            msg_info = "System error, can not create point folder!";
            return 12;
        }
		}
		if (-1 == access((config_map_path_ + save_scene_name + "/virtual_wall").c_str(), F_OK))
    	{
        if (-1 == mkdir((config_map_path_ + save_scene_name + "/virtual_wall").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            msg_info = "System error, can not create virtual wall folder!";
            return 13;
        }
		}
		if (-1 == access((config_map_path_ + save_scene_name + "/label").c_str(), F_OK))
    	{
		if (-1 == mkdir((config_map_path_ + save_scene_name + "/label").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            msg_info = "System error, can not create virtual wall folder!";
            return 14;
        }
		}
		if (-1 == access((config_map_path_ + save_scene_name + "/regions").c_str(), F_OK))
    	{
		if (-1 == mkdir((config_map_path_ + save_scene_name + "/regions").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            msg_info = "System error, can not create regions folder!";
            return 15;
        }
		}
		if (-1 == access((config_map_path_ + save_scene_name + "/fixpathes").c_str(), F_OK))
    	{
		if (-1 == mkdir((config_map_path_ + save_scene_name + "/fixpathes").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            msg_info = "System error, can not create fixpathes folder!";
            return 16;
        }
		}

    return 0;
}
int MapServer::save_map_to_file(const nav_msgs::OccupancyGridConstPtr map,  const std::string &save_scene_name, const std::string &save_fname, std::string &msg_info)
{

    ROS_DEBUG("Received a %d X %d map @ %.3f m/pixs", map->info.width,
              map->info.height, map->info.resolution);

    std::string mapdatafile = config_map_path_ + save_scene_name + "/map/" + save_fname + ".pgm";
    ROS_DEBUG("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        msg_info = "System error, couldn't save map!";
        return 13;
    }


    fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);
    for (unsigned int y = 0; y < map->info.height; y++)
    {
        for (unsigned int x = 0; x < map->info.width; x++)
        {
            unsigned int i = x + (map->info.height - y - 1) * map->info.width;
            if (map->data[i] == 0)
            { //occ [0,0.1)
                fputc(254, out);
            }
            else if (map->data[i] == +100)
            { //occ (0.65,1]
                fputc(000, out);
            }
            else
            { //occ [0.1,0.65]
                fputc(205, out);
            }
        }
    }

    fclose(out);

    std::string mapmetadatafile = config_map_path_ + save_scene_name + "/map/" + save_fname + ".yaml";
    ROS_DEBUG("Writing map occupancy data to %s", mapmetadatafile.c_str());
    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

    /*
			 	 resolution: 0.100000
			 	 origin: [0.000000, 0.000000, 0.000000]
			 	 #
			 	 negate: 0
			 	 occupied_thresh: 0.65
			 	 free_thresh: 0.196

     */

    geometry_msgs::Quaternion orientation = map->info.origin.orientation;
    tf::Matrix3x3 mat(
            tf::Quaternion(orientation.x, orientation.y, orientation.z,
                           orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    fprintf(yaml,
            "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            (save_fname + ".pgm").c_str(), map->info.resolution,
            map->info.origin.position.x, map->info.origin.position.y, yaw);

    fclose(yaml);

    std::string fname = (config_map_path_ + save_scene_name
            + "/point/" + save_fname + ".yaml");

    std::ofstream create(fname.c_str());
    create.close();

    fname = (config_map_path_ + save_scene_name
            + "/virtual_wall/" + save_fname + ".yaml");

    std::ofstream create1(fname.c_str());
    create1.close();

	fname = (config_map_path_ + save_scene_name
            + "/label/" + save_fname + ".yaml");

    std::ofstream create2(fname.c_str());
    create2.close();
    msg_info = "Map Save Succeed!";
    return 0;

    ROS_DEBUG("Done\n");

}

void MapServer::loadAllMaps(const std::string& scene)
{

    map_server::scene sce;
    sce.scene_name = scene;
    map_list_manager_->traverse_folder(sce);
    for (std::vector<std::string>::iterator it = sce.map_list.begin();
            it != sce.map_list.end(); it++)
    {
        std::string fname = (*it);
        boost::shared_ptr<nav_msgs::GetMap::Response> rep_ptr(new nav_msgs::GetMap::Response());
        ROS_DEBUG("find %s", fname.c_str());
        std::string map_path = config_map_path_ + scene + "/map/" + fname;
        int err = readPGM(map_path, rep_ptr);
        if (err != 0)
        {
            continue;
        }
        maps_[fname] = rep_ptr;

    }
}

int MapServer::readPGM(const std::string& fname,boost::shared_ptr<nav_msgs::GetMap::Response> &rep_ptr)
{
    std::string mapfname = "";

    std::string exd[] =
    { ".yaml", ".pgm" };

    int err;
    double origin[3];
    int negate;

    double occ_th, free_th;
    MapMode mode = TRINARY;
    std::string frame_id;

    private_nh_.param("frame_id", frame_id, std::string("map"));

    if (!deprecated)
    {
        ROS_DEBUG("Loading yaml from file \"%s\"", fname.c_str());
        if(0!=(err = readMapYAML(mapfname, fname + exd[0], res_, negate, occ_th,
                                 free_th, origin, mode)))
        {
            return err;
        }
    }
    else
    {
        private_nh_.param("negate", negate, 0);
        private_nh_.param("occupied_thresh", occ_th, 0.65);
        private_nh_.param("free_thresh", free_th, 0.196);
        mapfname = fname + exd[1];
        origin[0] = origin[1] = origin[2] = 0.0;
    }

    ROS_DEBUG("Loading map from image \"%s\"", mapfname.c_str());
    try
    {
        map_server::loadMapFromFile(rep_ptr.get(), mapfname.c_str(), res_, negate,
                                    occ_th, free_th, origin, mode);
    }catch(std::runtime_error & e)
    {
        ROS_ERROR("%s",e.what());
        return -1;
    }
    rep_ptr->map.info.map_load_time = ros::Time::now();
    rep_ptr->map.header.frame_id = frame_id;
    rep_ptr->map.header.stamp = ros::Time::now();
    ROS_DEBUG("stamp time 1: %f", rep_ptr->map.header.stamp.toSec());
    ROS_DEBUG("Read a %d X %d map @ %.3lf m/cell", rep_ptr->map.info.width,
              rep_ptr->map.info.height, rep_ptr->map.info.resolution);
    //meta_data_message_ = map_resp_.map.info;
    return 0;

}
int MapServer::readMapYAML(std::string &mapfname, const std::string &fname, double &res,
                           int &negate, double &occ_th, double &free_th, double* origin,
                           MapMode &mode)
{

    std::ifstream fin(fname.c_str());
    if (fin.fail())
    {
        ROS_ERROR("Map_server could not open %s.", fname.c_str());
        return -1;
    }
#ifdef HAVE_NEW_YAMLCPP
    // The document loading process changed in yaml-cpp 0.5.
    YAML::Node doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif

    try
    {
        doc["resolution"] >> res;
        ROS_DEBUG("res:%f", res);
    }
    catch (YAML::Exception  &e)
    {
        ROS_ERROR(
                "The map does not contain a resolution tag or it is invalid. %s",e.msg.c_str());
        return -1;;
    }
    try
    {
        doc["negate"] >> negate;
        ROS_DEBUG("negate:%d", negate);
    } catch (YAML::Exception  &e)
    {
        ROS_ERROR(
                "The map does not contain a negate tag or it is invalid. %s",e.msg.c_str());
        return -1;;
    }
    try
    {
        doc["occupied_thresh"] >> occ_th;
        ROS_DEBUG("occ_th:%f", occ_th);
    } catch (YAML::Exception  &e)
    {
        ROS_ERROR(
                "The map does not contain an occupied_thresh tag or it is invalid. %s",e.msg.c_str());
        return -1;;
    }
    try
    {
        doc["free_thresh"] >> free_th;
        ROS_DEBUG("free_thresh:%f", free_th);
    } catch (YAML::Exception  &e)
    {
        ROS_ERROR(
                "The map does not contain a free_thresh tag or it is invalid. %s",e.msg.c_str());
        return -1;;
    }
    try
    {
        std::string modeS = "";
        doc["mode"] >> modeS;

        if (modeS == "trinary")
            mode = TRINARY;
        else if (modeS == "scale")
            mode = SCALE;
        else if (modeS == "raw")
            mode = RAW;
        else
        {
            ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
            return -1;;
        }
    } catch (YAML::Exception &e)
    {
        ROS_DEBUG(
                "The map does not contain a mode tag or it is invalid... assuming Trinary");
        mode = TRINARY;
    }
    try
    {
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
    } catch (YAML::Exception &e)
    {
        ROS_ERROR(
                "The map does not contain an origin tag or it is invalid.");
        return -1;;
    }
    try
    {
        doc["image"] >> mapfname;
        // TODO: make this path-handling more robust
        if (mapfname.size() == 0)
        {
            ROS_ERROR("The image tag cannot be an empty string.");
            return -1;;
        }
        if (mapfname[0] != '/')
        {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
            ROS_DEBUG("mapfname:%s", mapfname.c_str());
        }
    } catch (YAML::Exception &e)
    {
        ROS_ERROR(
                "The map does not contain an image tag or it is invalid.");
        return -1;;
    }
    fin.close();

    return 0;
}
int MapServer::loadPointYAML(const std::string fname, std::vector<Point> &vec, std::string &msg_info)
{

    std::ifstream fin(fname.c_str());
    // check whether point yaml exist, if not, create it.
    if (!fin)
    {
        std::ofstream create(fname.c_str());
        create.close();
        msg_info.append("Point file does not exist and has created a new file.");
        return 0;
    }

    YAML::Node doc = YAML::Load(fin);

    if (!doc)
    {
        msg_info.append("Fail to open the points file.");
        return 2;
    }
    if (doc.size()==0)
    {
        msg_info.append("No Points");
        return 0;
    }


    for (YAML::Node::iterator it = doc.begin(); it != doc.end(); ++it)
    {
        try{
            Point p;
            p.nm = (it)->first.Scalar();
            (it)->second["x"] >> p.x;
            (it)->second["y"] >> p.y;
            (it)->second["th"] >> p.th;
            (it)->second["type"] >> p.type;
            (it)->second["alias"]>> p.alias;

            vec.push_back(p);
        }
        catch (YAML::Exception  &e)
        {
            ROS_ERROR("%s",e.msg.c_str());
        }

    }

    msg_info.append("Load Point Succeeded");
    return 0;
}
int MapServer::getId(std::vector<int> id_vec)
{
    if(id_vec.empty())
    {  
        return 0;  
    }  
    int match = 0;  
    int maxMatch = id_vec.size();  
              
    while(match<maxMatch)
    {  
        if(id_vec[match]==match+1)
        {  
            match++;  
        }
        else if(id_vec[match]<=match || id_vec[match]>maxMatch || id_vec[id_vec[match]-1] == id_vec[match])
        {  
            maxMatch--;  
            id_vec[match] = id_vec[maxMatch];  
        }
        else 
        {  
            int temp = id_vec[match];  
            id_vec[match] = id_vec[temp-1];  
            id_vec[temp-1] = temp;  
        }  
    }  
    return match;  
}


int MapServer::writePointYAML(const std::string fname, Point &p, std::string &msg_info)
{
    std::ifstream fin(fname.c_str());

    if (!fin)
    {
        std::ofstream create(fname.c_str());
        create.close();
        fin.open(fname.c_str());
    }

    YAML::Node doc = YAML::Load(fin);
    if (!doc)
    {
        msg_info.append("Fail to open the points file.");
        return 2;
    }
    std::vector<int> id_vec;

    for (YAML::Node::iterator it = doc.begin(); it != doc.end(); ++it)
    {
        try
        {
            double k = it->first.as<int>()+1.0;
            ROS_INFO("k:%f",k);
            id_vec.push_back(k);
        }
        catch (YAML::Exception  &e)
        {
            ROS_ERROR("%s",e.msg.c_str());
        }

    }
    int id = getId(id_vec);
    
    //std::stringstream stream;
    //stream << current_max_id;
    //stream >> p.nm;

    //p.nm = current_max_id.Scalar();
    int2str(id,p.nm);
    doc[p.nm]["alias"] = p.alias;
    doc[p.nm]["x"] = p.x;
    doc[p.nm]["y"] = p.y;
    doc[p.nm]["th"] = p.th;
    doc[p.nm]["type"] = p.type;
    //doc[CURRENT_MAX_ID] = ++current_max_id;
    std::ofstream fout(fname.c_str());

    fout << doc;
    fout.close();
    msg_info = "Adding Point Succeed!";
    return 0;

}
int MapServer::queryPointYAML(const std::string fname, Point &p, std::string &msg_info)
{
    std::ifstream fin(fname.c_str());

    if (!fin)
    {
        ROS_ERROR("fail to open the file %s tag or it is invalid.",fname.c_str());
        msg_info.append("Fail to open the point file!");
        return 2;
    }

    YAML::Node doc = YAML::Load(fin);
    if (!doc)
    {
        msg_info.append("Fail to open the points file!");
        return 2;
    }
    YAML::Node::iterator it;
    for (it = doc.begin(); it != doc.end(); ++it)
    {
        try
        {
            if(p.nm == (it)->first.Scalar())break;
        }
        catch (YAML::Exception  &e)
        {
            ROS_ERROR("%s",e.msg.c_str());
        }
    }


    if(it!=doc.end())
    {
        try
        {
            (it)->second["x"] >> p.x;
            (it)->second["y"] >> p.y;
            (it)->second["th"] >> p.th;
            (it)->second["type"] >> p.type;
            (it)->second["alias"]>> p.alias;

        }
        catch (YAML::Exception &e)
        {
            ROS_ERROR("%s",e.msg.c_str());
            msg_info = "Point not found!";
            return 3;
        }

    }
    else
    {
        msg_info = "Point not found!";
        return 3;
    }
    msg_info.append("Point Query Succeeded!");
    return 0;

}
int MapServer::editPointYAML(const std::string fname, const std::string point_name_str, const Point &p, std::string &msg_info)
{
    std::ifstream fin(fname.c_str());
    if (!fin)
    {
        ROS_ERROR("fail to open the file %s tag or it is invalid.",fname.c_str());
        msg_info.append("Fail to open the point file");
        return 2;
    }
    YAML::Node doc = YAML::Load(fin);
    if (!doc)
    {
        msg_info.append("Fail to open the points file!");
        return 2;
    }
    YAML::Node::iterator it;
    for (it = doc.begin(); it != doc.end(); ++it)
    {
        try
        {
            if(point_name_str == (it)->first.Scalar())break;
        }
        catch (YAML::Exception &e)
        {
            ROS_ERROR("%s",e.msg.c_str());
        }

    }

    if(it!=doc.end())
    {


        try
        {
            if(point_name_str != p.nm)
            {
                (it)->first = p.nm;
            }
            (it)->second["alias"] = p.alias;
            (it)->second["x"] = p.x;
            (it)->second["y"] = p.y;
            (it)->second["th"] = p.th;
            (it)->second["type"] = p.type;
        }
        catch (YAML::Exception &e)
        {
            doc.remove(p.nm);
            msg_info = "Point not found!";
            return 3;
        }
    }
    else
    {
        msg_info = "Point not found!";
        return 3;
    }
    std::ofstream fout(fname.c_str());
    fout << doc;
    fout.close();
    msg_info.append("Edit Succeeded");
    return 0;
}

int MapServer::deletePointYAML(const std::string fname, const Point &p, std::string &msg_info)
{
    std::ifstream fin(fname.c_str());
    if (!fin)
    {
        msg_info.append("Fail to open the point file");
        return 2;
    }
    YAML::Node doc = YAML::Load(fin);
    if(doc.size()==0)
    {

        msg_info.append("No Point Exist!");
        return 0;
    }
    if(doc.remove(p.nm))
    {
        msg_info.append("Delete Succeeded");
        std::ofstream fout(fname.c_str());
        fout << doc;
        fout.close();
        return 0;
    }

    msg_info.append("Delete Failed");
    return 3;
}


int MapServer::mapRenameDataExtract(Jptr jptr,std::string &scene_name,std::string &old_name_str,std::string &new_name_str, std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);
    if ((*data)[ORIGIN_NAME].isNull()|| (*data)[NEW_NAME].isNull()|| (*data)[SCENE_NAME].isNull())
    {
        msg_info.append("The json tag is invalid.");
        return 1;
    }
    scene_name = (*data)[SCENE_NAME].asString();
    old_name_str = (*data)[ORIGIN_NAME].asString();
    new_name_str = (*data)[NEW_NAME].asString();
    return 0;
}

int MapServer::mapSceneRenameDataExtract(Jptr jptr,std::string &old_scene_str, std::string &new_scene_str, std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);
    if ((*data)[ORIGIN_SCENE_NAME].isNull()|| (*data)[NEW_SCENE_NAME].isNull())
    {
        msg_info.append("The json tag is invalid.");
        return 1;
    }

    old_scene_str = (*data)[ORIGIN_SCENE_NAME].asString();
    new_scene_str = (*data)[NEW_SCENE_NAME].asString();
    return 0;

}
int MapServer::mapDeleteDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);
    if ((*data)[SCENE_NAME].isNull()|| (*data)[MAP_NAME].isNull())
    {
        msg_info.append("The json tag is invalid.");
        return 1;
    }

    scene_name_str = (*data)[SCENE_NAME].asString();
    map_name_str = (*data)[MAP_NAME].asString();
    return 0;


}
int MapServer::mapSceneDeleteDataExtract(Jptr jptr,std::string &scene_name_str, std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);
    if ((*data)[SCENE_NAME].isNull())
    {
        msg_info.append("The json tag is invalid.");
        return 1;
    }

    scene_name_str = (*data)[SCENE_NAME].asString();
    return 0;

}
int MapServer::mapSaveDataExtract(Jptr jptr,std::string &save_scene_name, std::string &save_fname, std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);
    if ((*data)[SCENE_NAME].isNull()|| (*data)[MAP_NAME].isNull())
    {
        msg_info.append("The json tag is invalid.");
        return 1;
    }

    save_scene_name = (*data)[SCENE_NAME].asString();
    save_fname = (*data)[MAP_NAME].asString();
    return 0;


}
int MapServer::mapLoadDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);
    if ((*data)[SCENE_NAME].isNull()|| (*data)[MAP_NAME].isNull())
    {
        msg_info.append("The json tag is invalid.");
        return 1;
    }

    scene_name_str = (*data)[SCENE_NAME].asString();
    map_name_str = (*data)[MAP_NAME].asString();
    return 0;

}

int MapServer::pointAddDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, Point &p,std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);

    if ((*data)[POINT_NAME].isNull()
            || (*data)[SCENE_NAME].isNull()
            || (*data)[MAP_NAME].isNull()
            || (*data)[POINT_ALIAS].isNull() || !(*data)[X].isNumeric()
            || !(*data)[Y].isNumeric() || !(*data)[TH].isNumeric()
            || !(*data)[POINT_TYPE].isInt())
    {
        msg_info.append("The json does not contain an incomplete tag or it is invalid.");
        return 1;
    }
    scene_name_str = (*data)[SCENE_NAME].asString();
    map_name_str= (*data)[MAP_NAME].asString();
    p.nm =(*data)[POINT_NAME].asString();
    p.alias =(*data)[POINT_ALIAS].asString();
    p.x = (*data)[X].asDouble();
    p.y = (*data)[Y].asDouble();
    p.th = (*data)[TH].asDouble();
    p.type = (*data)[POINT_TYPE].asInt();
    return 0;
}

int MapServer::pointLoadDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);
    if ((*data)[SCENE_NAME].isNull()|| (*data)[MAP_NAME].isNull())
    {
        msg_info.append("The json does not contain an incomplete tag or it is invalid.");
        return 1;
    }
    scene_name_str = (*data)[SCENE_NAME].asString();
    map_name_str= (*data)[MAP_NAME].asString();
    return 0;
}
int MapServer::pointQueryDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str,Point &p, std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);
    if ((*data)[SCENE_NAME].isNull()|| (*data)[MAP_NAME].isNull()||(*data)[POINT_NAME].isNull())
    {
        msg_info.append("The json does not contain an incomplete tag or it is invalid.");
        return 1;
    }
    scene_name_str = (*data)[SCENE_NAME].asString();
    map_name_str= (*data)[MAP_NAME].asString();
    p.nm =(*data)[POINT_NAME].asString();
    return 0;
}
int MapServer::pointEditDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str,std::string &point_name_str, Point &p,std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);
    if ((*data)[POINT_NAME].isNull()
            || (*data)[SCENE_NAME].isNull()
            || (*data)[MAP_NAME].isNull()||(*data)[NEW_POINT_NAME].isNull()
            || (*data)[NEW_POINT_ALIAS].isNull() || !(*data)[NEW_X].isNumeric()
            || !(*data)[NEW_Y].isNumeric() || !(*data)[NEW_TH].isNumeric()
            || !(*data)[NEW_POINT_TYPE].isInt())
    {
        msg_info.append("The json does not contain an incomplete tag or it is invalid.");
        return 1;
    }
    scene_name_str = (*data)[SCENE_NAME].asString();
    map_name_str= (*data)[MAP_NAME].asString();
    point_name_str = (*data)[POINT_NAME].asString();
    p.nm =(*data)[NEW_POINT_NAME].asString();
    p.alias =(*data)[NEW_POINT_ALIAS].asString();
    p.x = (*data)[NEW_X].asDouble();
    p.y = (*data)[NEW_Y].asDouble();
    p.th = (*data)[NEW_TH].asDouble();
    p.type = (*data)[NEW_POINT_TYPE].asInt();
    return 0;

}

int MapServer::pointDeleteExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str,Point &p, std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);

    if (((*data)[POINT_NAME].isNull()
            || (*data)[SCENE_NAME].isNull()
            || (*data)[MAP_NAME].isNull()))
    {
        msg_info.append("The json does not contain an incomplete tag or it is invalid.");
        return 1;
    }
    scene_name_str = (*data)[SCENE_NAME].asString();
    map_name_str= (*data)[MAP_NAME].asString();
    p.nm =(*data)[POINT_NAME].asString();
    return 0;

}


bool MapServer::isMapLoad()
{
    return map_loaded;
}
void MapServer::setMapLoad(bool load)
{
    map_loaded = load;
}


bool MapServer::queryAllPoints(map_server::query_all_points::Request &req,
                                map_server::query_all_points::Response &res)

{   
    map_server::scene sce;
    std::string msg_info;
    std::string path;
    Json::Reader reader;
    Json::Value value;
    Json::Value result;
    Json::Value pointdata;
    std::string test = req.scene_name;
    //sce.scene_name = req.scene_name;
	ROS_ERROR("scene_name:%s received",test.c_str());
    if(reader.parse(test,value))
    {
       if(value["scene_name"].isNull())
       {          
		   ROS_ERROR("scene_name value is empty");
       }
       else
       {   
           ROS_ERROR("scene_name received confirm");
           sce.scene_name = value[SCENE_NAME].asString();
           map_list_manager_->traverse_folder(sce);
           int k = 0;
           for(std::vector<std::string>::iterator iter = sce.map_list.begin();iter != sce.map_list.end();iter++)
            {
                path = (config_map_path_ + sce.scene_name + "/point/" + *iter + ".yaml");
                ROS_ERROR("path:%s",path.c_str());
                //dataArray =  (*jptr)[DATA];
                std::vector<Point> vec;
                if(0 != loadPointYAML(path,vec,msg_info))
                {
                    return false;
                }
                for(unsigned int i = 0; i < vec.size(); ++i)
                {
                    pointdata[k][POINT_NAME] = vec[i].nm;
                    pointdata[k][POINT_ALIAS] = vec[i].alias;
                    pointdata[k][POINT_TYPE] = vec[i].type;
                    pointdata[k][X] = vec[i].x;
                    pointdata[k][Y] = vec[i].y;
                    pointdata[k][TH]= vec[i].th;
                    pointdata[k][MAP_NAME] = *iter;
                    k++;
                }

            }

                // map_server::point_map point;
                // for(unsigned int i = 0; i < vec.size(); i++)
                // {
                //     point.index = atoi(vec[i].nm.c_str());
                //     point.type = vec[i].type;
                //     point.alias = vec[i].alias;
                //     point.x = vec[i].x;
                //     point.y = vec[i].y;
                //     point.th = vec[i].th;
                //     point.map_name = *iter;
                //     res.all_points.push_back(point);
                // }
           Json::StyledWriter fast;
           res.all_points = fast.write(pointdata);
           return true;
       }
    }
    return true;


   

}

bool MapServer::queryUwbPoints(map_server::query_uwb::Request  &req,
                                map_server::query_uwb::Response &res)
{
    if(!isMapLoad())
    {
        return false;
    }

    std::vector<Point> vec;
    std::string msg_info;
    std::string path = (config_map_path_ + this->current_scene_ + "/point/" + this->current_map_ + ".yaml");
    if(0 != loadPointYAML(path,vec,msg_info))
    {
        return false;
    }

    map_server::point point;
    for(unsigned int i = 0; i < vec.size(); i++)
    {
        if(vec[i].type == UWB_POINT_TYPE)
        {
            point.index = atoi(vec[i].nm.c_str());
            point.type = vec[i].type;
            point.alias = vec[i].alias;
            point.x = vec[i].x;
            point.y = vec[i].y;
            point.th = vec[i].th;
            res.uwb_points.push_back(point);
        }
    }
    return true;
}

#endif /* MAP_SERVER_INCLUDE_MAP_SERVER_MAP_SERVER_FUNCTIONS_HPP_ */
