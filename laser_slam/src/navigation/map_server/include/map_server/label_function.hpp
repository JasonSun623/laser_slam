/*
 * label_function.hpp
 *
 *  Created on: May 22, 2017
 *      Author: Xumi
 */

#ifndef MAP_SERVER_INCLUDE_LABEL_FUNCTION_HPP_
#define MAP_SERVER_INCLUDE_LABEL_FUNCTION_HPP_

#include "map_server/map_server_mrobot.h"
#include "map_server/virtual_wall_function.hpp"
#include <ar_track_alvar_msgs/AlvarId.h>
#include <boost/tuple/tuple.hpp>

using namespace map_server;

void Label::reset()
{
	x = 0.0;
	y = 0.0;
	z = 0.0;
	th = 0.0;
	id = 0;
}

int MapServer::saveLabel2Yaml(const std::string &scene_name,const std::string &map_name)
{
	std::string scene_name_str = scene_name;
	std::string map_name_str = map_name;

	std::string path = (config_map_path_ + scene_name_str + "/label/" + map_name_str + ".yaml");
	return addLabelYAML(path);
}
int MapServer::addLabelYAML(const std::string fname)
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
		ROS_INFO("Fail to open the label file.");
		return 1;
	}
	doc[label2map_.id][X] = label2map_.x;
	doc[label2map_.id][Y] = label2map_.y;
	doc[label2map_.id][Z] = label2map_.z;
	doc[label2map_.id][TH] = label2map_.th;
	std::ofstream fout(fname.c_str());

	fout << doc;
	fout.close();
	//msg_info = "Adding Label Succeed!";
	return 0;

}

int MapServer::labelVectorInit(const std::string &scene_name,const std::string &map_name)
{
	ROS_INFO("label initial vec");	
	std::string scene_name_str = scene_name;
	std::string map_name_str = map_name;
	boost::get<0>(label_save_info_) = scene_name_str;
	boost::get<1>(label_save_info_) = map_name_str;
    if(0 != access((config_map_path_ + scene_name_str+"/label").c_str(),0))
    {
        ROS_INFO("No label directory");
        mkdir((config_map_path_ + scene_name_str+"/label").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
	std::string path = (config_map_path_ + scene_name_str + "/label/" + map_name_str + ".yaml");
	return loadLabelYAML(path);
}

int MapServer::loadLabelYAML(const std::string fname)
{	
	ROS_INFO("MapServer::loadLabelYAML");
	std::ifstream fin(fname.c_str());
	if (!fin)
	{
		std::ofstream create (fname.c_str());
		create.close();
		fin.open(fname.c_str());
	}
	//ROS_ERROR("2");
	YAML::Node doc = YAML::Load(fin);
	
	if (!doc)
	{
		ROS_ERROR("Fail to open the label file.");
		return 1;
	}
	label_vec_.clear();
	label_.reset();
	label2map_.reset();
	last_rcv_id_ = -1;
	for (YAML::Node::iterator it = doc.begin(); it != doc.end(); ++it)
	{
		try{
			Label l;
			l.id = ((it)->first).as<int>();
			(it)->second["x"] >> l.x;
			(it)->second["y"] >> l.y;
			(it)->second["z"] >> l.z;
			(it)->second["th"] >>l.th;
			label_vec_.push_back(l);
			ROS_ERROR("3");
		}
		catch (YAML::Exception  &e)
		{
			ROS_ERROR("Loading labels failed! %s",e.msg.c_str());
			return 1;
		}

	}

	//fout << doc;
	fin.close();
	ROS_INFO("Loading Label Succeed!");
	return 0;

}

void MapServer::label_save(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{	
	ROS_INFO("Lable save cmd callback");
	std::string scene_name_str, map_name_str;
	if(0!=(err=labelSaveExtract(jptr,scene_name_str,map_name_str,msg_info))) return;
	//boost::get<0>(label_save_info_) = scene_name_str;
	//boost::get<1>(label_save_info_) = map_name_str;
	boost::get<2>(label_save_info_) = true;
	dataArray =  (*jptr)[DATA];
	//if(0!=(err=deletePointYAML(path,p,msg_info)))return;
}

int MapServer::labelSaveExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str,std::string &msg_info)
{
    Json::Value* data =  &((*jptr)[DATA]);

    if (((*data)[SCENE_NAME].isNull()|| (*data)[MAP_NAME].isNull()))
    {
        msg_info.append("The json does not contain an incomplete tag or it is invalid.");
        return 1;
    }
    scene_name_str = (*data)[SCENE_NAME].asString();
    map_name_str= (*data)[MAP_NAME].asString();
    return 0;

}

void MapServer::label_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{	
	ROS_INFO("Lable delete cmd callback");
	std::string scene_name_str, map_name_str;
	if(0!=(err=labelDeleteDataExtract(jptr,scene_name_str,map_name_str,msg_info))) return;
    std::string label_yaml = config_map_path_ + scene_name_str + "/label/" + map_name_str + ".yaml";
	remove(label_yaml.c_str());
	std::ofstream create(label_yaml.c_str());
	create.close();
	dataArray =  (*jptr)[DATA];
	msg_info.append("Delete Label Succeed!");
	err = 0;
	//if(0!=(err=deletePointYAML(path,p,msg_info)))return;
}

int MapServer::labelDeleteDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info)
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


int MapServer::getMostIdFromMap(std::map<int, int> &id_to_num)
{
    int id = (id_to_num.begin())->first;
    int most = (id_to_num.begin())->second;
    for(auto iter = id_to_num.begin(); iter != id_to_num.end(); ++iter)
    {
        if(iter->second > most)
        {
            id = iter->first;
            most = iter->second;
        }
    }
    return id;
}


/*

void MapServer::label_add(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	std::string scene_name_str, map_name_str;
	if(0!=(err=labelAddDataExtract(jptr,scene_name_str,map_name_str,msg_info))) return;
	std::string path = (config_map_path_ + scene_name_str + "/label/" + map_name_str + ".yaml");
	dataArray =  (*jptr)[DATA];
	dataArray[ID] = l.id;
	if(0!=(err=writeLabelYAML(path,label2map_,msg_info)))return ;	
}
void MapServer::label_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	std::string scene_name_str, map_name_str;
	Label l = label_;
	if(0!=(err=labelDeleteExtract(jptr,scene_name_str,map_name_str,l,msg_info))) return;

	std::string path = (config_map_path_ + scene_name_str + "/label/" + map_name_str + ".yaml");
	dataArray =  (*jptr)[DATA];
	if(0!=(err=deletelabelYAML(path,l,msg_info)))return;	
}
void MapServer::label_load(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{

	std::string scene_name_str, map_name_str;
	std::vector<Point> vec;
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
int MapServer::writelabelYAML(const std::string fname, Label label2map_, std::string &msg_info)
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
		msg_info.append("Fail to open the label file.");
		return 2;
	}
	YAML::Node::iterator it;
	for (it = doc.begin(); it != doc.end(); ++it)
	{
		try
		{
			if(label2map_.it == (it)->first.Scalar())break;
		}
		catch (YAML::Exception  &e)
		{
			ROS_ERROR("%s",e.msg.c_str());
		}
	}

	if(!fout)
	{
		msg_info = "Adding Virtual Wall Failed!";
		return 14;
	}

	fout << doc;
	fout.close();
	msg_info = "Adding Virtual Wall Succeed!";
	return 0;

}
int MapServer::deletelabelYAML(const std::string fname, Label l, std::string &msg_info)
{
	std::ifstream fin(fname.c_str());
	if (!fin)
	{
		msg_info.append("Fail to open the virtual wall file");
		return 2;
	}
	YAML::Node doc = YAML::Load(fin);
	if(doc.size()==0)
	{

		msg_info.append("No Virtual Wall Exists!");
		return 0;
	}

	for(std::vector<boost::shared_ptr<Shape> >::const_iterator it = vec.begin(); it != vec.end(); ++it)
	{
		if(!(*it)->deleteInYAML(doc))
		{
			std::string sid;
			int2str((*it)->id,sid);
			msg_info.append("Delete "+(*it)->name + " id: " + sid +" failed!");
			return 3;

		}
	}



	msg_info.append("Delete Succeeded");
	std::ofstream fout(fname.c_str());
	fout << doc;
	fout.close();
	return 0;

	//msg_info.append("Delete Failed");
	//return 3;
}
int MapServer::labelAddDataExtract(Jptr jptr,std::string &scene_name_str,std::string &map_name_str, std::vector<boost::shared_ptr<Shape > > &vec,std::string &msg_info)
{
	Json::Value* data =  &((*jptr)[DATA]);
	if ((*data)[SCENE_NAME].isNull()|| (*data)[MAP_NAME].isNull()||
			((*data)[POLYGONS].isNull()&&(*data)[CIRCLES].isNull()))
	{
		msg_info.append("The json does not contain an incomplete tag or it is invalid.");
		return 1;
	}

	scene_name_str = (*data)[SCENE_NAME].asString();
	map_name_str= (*data)[MAP_NAME].asString();
	if(	!(*data)[POLYGONS].isNull())
	{
		Json::Value polygons = (*data)[POLYGONS];
		for (Json::Value::iterator it = polygons.begin(); it != polygons.end(); ++it)
		{
			boost::shared_ptr<Polygon> p = boost::make_shared<Polygon>();

			p->convertFromJson(*(it));
			vec.push_back(p);
		}
	}

	if(!(*data)[CIRCLES].isNull())
	{
		Json::Value circles = (*data)[CIRCLES];
		for (Json::Value::iterator it = circles.begin(); it != circles.end(); ++it)
		{
			boost::shared_ptr<Circle> c = boost::make_shared<Circle>();
			c->convertFromJson(*(it));
			vec.push_back(c);
		}
	}
	ROS_ERROR("size %d", (int)vec.size());

	return 0;
}
int MapServer::labelDeleteExtract(Jptr jptr,std::string &scene_name_str,std::string &map_name_str, std::vector<boost::shared_ptr<Shape> > &vec, std::string &msg_info)
{
	Json::Value* data =  &((*jptr)[DATA]);

	if ((((*data)[CIRCLE].isNull()&&(*data)[POLYGON].isNull())
			|| (*data)[SCENE_NAME].isNull()
			|| (*data)[MAP_NAME].isNull()))
	{
		msg_info.append("The json does not contain an incomplete tag or it is invalid.");
		return 1;
	}

	scene_name_str = (*data)[SCENE_NAME].asString();

	map_name_str= (*data)[MAP_NAME].asString();

	if(!(*data)[CIRCLE].isNull())
	{
		boost::shared_ptr<Circle> c = boost::make_shared<Circle>();
		c->id =(*data)[CIRCLE][ID].asInt();
		vec.push_back(c);
	}
	if(!(*data)[POLYGON].isNull())
	{
		boost::shared_ptr<Polygon> p = boost::make_shared<Polygon>();
		p->id =(*data)[POLYGON][ID].asInt();
		vec.push_back(p);
	}

	return 0;

}
*/
#endif /* MAP_SERVER_INCLUDE_MAP_SERVER_label_FUNCTION_HPP_ */
