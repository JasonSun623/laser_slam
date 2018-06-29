/*
 * virtual_wall_function.hpp
 *
 *  Created on: Apr 28, 2017
 *      Author: tomzhang
 */

#ifndef MAP_SERVER_INCLUDE_MAP_SERVER_VIRTUAL_WALL_FUNCTION_HPP_
#define MAP_SERVER_INCLUDE_MAP_SERVER_VIRTUAL_WALL_FUNCTION_HPP_

#include "map_server/map_server_mrobot.h"

using namespace map_server;

void map_server::int2str(const int &int_temp, std::string &string_temp)
{
	std::stringstream stream;
	stream << int_temp;
	stream >> string_temp;
}

void MapServer::virtual_wall_load(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{

	std::string scene_name_str, map_name_str;
	if(0!=(err=virtualWallLoadDataExtract(jptr,scene_name_str,map_name_str,msg_info)))return;
	std::vector<boost::shared_ptr<Shape> > shape_vec;

	//std::vector<Point> vec;
	std::string path = (config_map_path_ + scene_name_str + "/virtual_wall/" + map_name_str + ".yaml");
	dataArray[MAP_NAME] = map_name_str;
	dataArray[SCENE_NAME] = scene_name_str;
	if(0 != access((config_map_path_ + scene_name_str+"/virtual_wall").c_str(),0))
    {
        ROS_INFO("No virtual_wall directory");
        mkdir((config_map_path_ + scene_name_str+"/virtual_wall").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
	if(0!=(err=loadVirtualWallYAML(path,shape_vec,msg_info)))return;
	sensor_msgs::PointCloud2 wall_cloud;
	std::vector<Point>  points;
	if(0!=(err=createVirtualWallPointCloud(shape_vec,wall_cloud,step_,msg_info))) return;
	this->virtual_wall_pub.publish(wall_cloud);

	for(std::vector<boost::shared_ptr<Shape> >::iterator it = shape_vec.begin(); it != shape_vec.end(); ++it)
	{
		(*it)->convertToJson(dataArray);
	}
}
void MapServer::virtual_wall_add(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	std::string scene_name_str, map_name_str;
	std::vector<boost::shared_ptr<Shape > > shape_vec;

	if(0!=(err=virtualWallAddDataExtract(jptr,scene_name_str,map_name_str, shape_vec,msg_info))) return;
	std::string path = (config_map_path_ + scene_name_str + "/virtual_wall/" + map_name_str + ".yaml");
	std::vector<int> shapes_id;
	dataArray[MAP_NAME] = map_name_str;
	dataArray[SCENE_NAME] = scene_name_str;
	if(0!=(err=writeVirtualWallYAML(path, shape_vec,msg_info,shapes_id)))return ;
	//dataArray =  (*jptr)[DATA];
	for(std::vector<boost::shared_ptr<Shape> >::iterator it = shape_vec.begin(); it != shape_vec.end(); ++it)
	{
		(*it)->convertToJson(dataArray);
	}


}
void MapServer::virtual_wall_delete(Jptr jptr, Json::Value &dataArray, int &err, std::string & msg_info)
{
	std::string scene_name_str, map_name_str;
	std::vector<boost::shared_ptr<Shape> >vec;

	if(0!=(err=virtualWallDeleteExtract(jptr,scene_name_str,map_name_str,vec,msg_info))) return;

	std::string path = (config_map_path_ + scene_name_str + "/virtual_wall/" + map_name_str + ".yaml");
	dataArray =  (*jptr)[DATA];
	if(0!=(err=deleteVirtualWallYAML(path,vec,msg_info)))return;
	
}




int MapServer::loadVirtualWallYAML(const std::string fname, std::vector<boost::shared_ptr<Shape> > &vec, std::string &msg_info)
{
	std::ifstream fin(fname.c_str());
	// check whether point yaml exist, if not, create it.
	if (!fin)
	{
		std::ofstream create(fname.c_str());
		create.close();
		msg_info.append("Virtual Wall file does not exist and has created a new file.");
		return 0;
	}

	YAML::Node doc = YAML::Load(fin);

	if (!doc)
	{
		msg_info.append("Fail to open the virtual wall file.");
		return 2;
	}
	if (doc.size()==0)
	{
		msg_info.append("No Virtual Walls");
		return 0;
	}

	YAML::Node circles = doc["circles"];
	YAML::Node polygons = doc["polygons"];

	for (YAML::Node::const_iterator it = circles.begin(); it != circles.end(); ++it)
	{
		try{
			boost::shared_ptr<Circle> c = boost::make_shared<Circle>();
			c->convertFromYAML(it);
			vec.push_back(c);
		}
		catch (YAML::Exception  &e)
		{
			ROS_ERROR("%s",e.msg.c_str());
		}

	}

	for (YAML::Node::const_iterator it = polygons.begin(); it != polygons.end(); ++it)
	{
		try{
			boost::shared_ptr<Polygon> p = boost::make_shared<Polygon>();
			p->convertFromYAML(it);
			vec.push_back(p);
		}
		catch (YAML::Exception  &e)
		{
			ROS_ERROR("%s",e.msg.c_str());
		}

	}

	msg_info.append("Load Virtual Wall Succeeded");
	return 0;
}
int MapServer::writeVirtualWallYAML(const std::string fname, std::vector<boost::shared_ptr<Shape> > &vec, std::string &msg_info, std::vector<int> shapes_id)
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
		msg_info.append("Fail to open the virtual wall file.");
		return 2;
	}
	if (!doc[CURRENT_MAX_ID])
	{
		doc[CURRENT_MAX_ID] = 0;
	}
	int current_max_id;

	doc[CURRENT_MAX_ID] >> current_max_id;



	for(std::vector<boost::shared_ptr<Shape> >::iterator it = vec.begin(); it != vec.end(); ++it)
	{
		(*it)->id = ++current_max_id;
		shapes_id.push_back(current_max_id);
		(*it)->convertToYAML(doc);
	}

	doc[CURRENT_MAX_ID] = current_max_id;

	std::ofstream fout(fname.c_str());
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
int MapServer::deleteVirtualWallYAML(const std::string fname, std::vector<boost::shared_ptr<Shape> > &vec, std::string &msg_info)
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


int MapServer::virtualWallAddDataExtract(Jptr jptr,std::string &scene_name_str,std::string &map_name_str, std::vector<boost::shared_ptr<Shape > > &vec,std::string &msg_info)
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

int MapServer::virtualWallDeleteExtract(Jptr jptr,std::string &scene_name_str,std::string &map_name_str, std::vector<boost::shared_ptr<Shape> > &vec, std::string &msg_info)
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

int MapServer::virtualWallLoadDataExtract(Jptr jptr,std::string &scene_name_str, std::string &map_name_str, std::string &msg_info)
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

int MapServer::createVirtualWallPointCloud(std::vector<boost::shared_ptr<Shape> > &in, sensor_msgs::PointCloud2 &cloud_out, const double &step, std::string &msg_info)
{
	if(step>=0.0)
	{
		std::vector<Point>  all_points;

		for(std::vector<boost::shared_ptr<Shape> >::iterator it = in.begin();it != in.end(); ++it)
		{
			std::vector<Point>  points;
			(*it)->getInterpolationPoints(points,step);
			if(points.size()>0)
			{
				all_points.insert(all_points.end(),points.begin(),points.end());
			}
		}

		if(all_points.size()>0)
		{
			int offset = 0;
			cloud_out.header.frame_id = "map";
			cloud_out.header.stamp = ros::Time::now();
			cloud_out.height = 1;
			cloud_out.width  = all_points.size ();
			cloud_out.fields.resize (4);
			cloud_out.fields[0].name = "x";
			cloud_out.fields[0].offset = offset;
			cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
			cloud_out.fields[0].count = cloud_out.width;
			cloud_out.fields[1].name = "y";
			cloud_out.fields[1].offset = 4;
			cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
			cloud_out.fields[1].count = cloud_out.width;
			cloud_out.fields[2].name = "z";
			cloud_out.fields[2].offset = 8;
			cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
			cloud_out.fields[2].count = cloud_out.width;
			cloud_out.fields[3].name = "intensity";
			cloud_out.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
			cloud_out.fields[3].offset = 12;
			cloud_out.fields[3].count = cloud_out.width;

			cloud_out.point_step = 16;
			cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
			cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
			cloud_out.is_dense = false;


			for(size_t i = 0; i < all_points.size (); ++i)
			{
				float *pstep = (float*)&cloud_out.data[i * cloud_out.point_step];
				pstep[0] = (float)(all_points[i].x);
				pstep[1] = (float)(all_points[i].y);


			}

			return 0;
		}
		return 0;
	}
	msg_info = "System parameter error, create virtual wall failed! ";
	return 4;
}

void Circle::getInterpolationPoints(std::vector<Point>  &points, double step )
{

	if(radius>0.0)
	{
		double delta_th;
		delta_th = std::asin(step/radius/2.0)*2.0;
		int size = std::floor(2.0*M_PI/delta_th) + 1;

		double x_r,y_r;
		points.reserve(size);
		for(double th = 0; th<2.0*M_PI;th+=delta_th)
		{
			x_r = x + radius*std::sin(th);
			y_r = y + radius*std::cos(th);
			Point p;
			p.x = x_r;
			p.y = y_r;
			points.push_back(p);
		}

	}


}
void Polygon::getInterpolationPoints(std::vector<Point>  &points, double step )
{

	std::vector<Point>  points_;
	for(std::vector<Line>::iterator it = linesegments.begin();it!=linesegments.end();++it)
	{
		it->getInterpolationPoints(points_,step);
		points.insert(points.end(),points_.begin(),points_.end());
	}
}

void  Line::getInterpolationPoints(std::vector<Point>  &points, double step  )
{


	if(start_x == start_y && end_x == end_y)
	{
		Point p;
		p.x = start_x;
		p.y = start_y;
		points.push_back(p);

	}
	double x,y, k, b;
	if(std::fabs(start_x-end_x)>std::fabs(start_y-end_y))
	{
		double x_s,x_e;
		if(start_x<end_x)
		{
			x_s = start_x;
			x_e = end_x;
		}
		else
		{
			x_e = start_x;
			x_s = end_x;
		}
		k = (start_y-end_y)/(start_x-end_x);
		b = start_y - k*start_x;
		int size = std::floor((x_e-x_s)/step) + 1;

		points.reserve(size);
		for(double x = x_s;x<x_e;x+=step)
		{
			y = k*x+b;
			Point p;
			p.x = x;
			p.y = y;
			points.push_back(p);
		}

	}
	else
	{
		double  y_s,y_e;
		if(start_y<end_y)
		{
			y_s = start_y;
			y_e = end_y;
		}
		else
		{
			y_e = start_y;
			y_s = end_y;
		}
		k = (start_x-end_x)/(start_y-end_y);
		b = start_x - k*start_y;
		int size = std::floor((y_e-y_s)/step) + 1;

		points.reserve(size);
		for(double y = y_s;y<y_e;y+=step)
		{
			x = k*y+b;
			Point p;
			p.x = x;
			p.y = y;
			points.push_back(p);
		}
	}




}

void Polygon::convertToJson(Json::Value &dataArray)
{



	Json::Value polygon;
	polygon[ID] =  id;
	polygon[TAG] = tag;
	Json::Value lines;

	for(std::vector<Line>::iterator l = linesegments.begin(); l != linesegments.end(); ++l)
	{
		l->convertToJson(lines);
	}
	polygon[LINESEGMENTS] =  lines;
	dataArray[POLYGONS].append(polygon);

}
void Polygon::convertToYAML(YAML::Node &doc)
{
	std::string pid;
	YAML::Node polygon;

	int2str(id,pid);
	for(std::vector<Line>::iterator it = this->linesegments.begin(); it!= this->linesegments.end(); ++it)
	{
		it->convertToYAML(polygon);
	}
	polygon[TAG] = tag;
	doc[POLYGONS][pid] = polygon;

}
void Polygon::convertFromJson(const Json::Value &dataArray)
{
	tag = dataArray[TAG].asString();
	Json::Value lines = dataArray[LINESEGMENTS];
	int k = 0;
	for (Json::Value::iterator it = lines.begin(); it != lines.end();++k,++it)
	{
		Line l;
		l.id = k;
		l.convertFromJson(*it);
		linesegments.push_back(l);
	}

}

void Polygon::convertFromYAML(const YAML::Node::const_iterator &doc)
{
	try
	{
		doc->first>>id;
		doc->second[TAG]>>tag;
		for (YAML::Node::const_iterator it = doc->second[LINESEGMENTS].begin(); it != doc->second[LINESEGMENTS].end(); ++it)
		{
				Line l;
				l.convertFromYAML(it);
				linesegments.push_back(l);
		}
	}
	catch(YAML::Exception  &e)
	{
		throw e;
	}
}
bool Polygon::deleteInYAML(YAML::Node &doc)
{
	std::string sid;
	int2str(id,sid);
	return doc[POLYGONS].remove(sid);
}
void Circle::convertFromJson(const Json::Value &dataArray)
{

	tag = dataArray[TAG].asString();
	radius = dataArray[RADIUS].asDouble();
	x = dataArray[ORIGIN_X].asDouble();
	y = dataArray[ORIGIN_Y].asDouble();

	//ROS_ERROR("id:%i,radius:%f,x:%f,y:%f",id,radius,x,y);
}

void Circle::convertToJson(Json::Value &dataArray)
{
	Json::Value circle;
	circle[ORIGIN_X] = x;
	circle[ORIGIN_Y] = y;
	circle[ID] = id;
	circle[RADIUS] = radius;
	circle[TAG] = tag;
	dataArray[CIRCLES].append(circle);
}

void Circle::convertToYAML(YAML::Node &doc)
{
	std::string cid;
	YAML::Node circle;
	YAML::Node origin;
	map_server::int2str(id,cid);
	//ROS_ERROR("yuan id shi %s",cid.c_str());
	origin[X] = x;
	origin[Y] = y;
	circle[ORIGIN] = origin;
	circle[RADIUS] = radius;
	circle[TAG] = tag;
	//ROS_ERROR("x:%f,y:%f,radius:%f",it->x,it->y,it->radius);
	doc[CIRCLES][cid] = circle;

}
void Circle::convertFromYAML(const YAML::Node::const_iterator &doc)
{
	try
	{
		doc->first>>id;
		doc->second[TAG]>>tag;
		doc->second["radius"] >> radius;
		doc->second["origin"][X] >> x;
		doc->second["origin"][Y] >> y;
	}
	catch(YAML::Exception  &e)
	{
		throw e;
	}
}
bool Circle::deleteInYAML(YAML::Node &doc)
{
	std::string sid;
	int2str(id,sid);
	return doc[CIRCLES].remove(sid);

}
void Line::convertToJson(Json::Value &dataArray)
{
	Json::Value line;
	line[ID] =id;
	line[START_X] = start_x;
	line[START_Y] = start_y;
	line[END_X] = end_x;
	line[END_Y] = end_y;
	dataArray.append(line);
}

void Line::convertFromJson(const Json::Value &dataArray)
{
	//id = dataArray[ID].asInt();;
	start_x = dataArray[START_X].asDouble();
	start_y = dataArray[START_Y].asDouble();
	end_x =  dataArray[END_X].asDouble();
	end_y = dataArray[END_Y].asDouble();

}
void Line::convertToYAML(YAML::Node &doc)
{
	std::string lid;
	YAML::Node linesegment;
	int2str(id,lid);
	linesegment["start"][X] = start_x;
	linesegment["start"][Y] = start_y;
	linesegment["end"][X] = end_x;
	linesegment["end"][Y] = end_y;
	doc[LINESEGMENTS][lid] = linesegment;

}


void Line::convertFromYAML(const YAML::Node::const_iterator &doc)
{
	try
	{
		doc->first>>id;
		doc->second["start"][X]>>start_x;
		doc->second["start"][Y]>>start_y;

		doc->second["end"][X] >> end_x;
		doc->second["end"][Y] >> end_y;
	}
	catch(YAML::Exception  &e)
	{
		throw e;
	}
}

bool Line::deleteInYAML(YAML::Node &doc)
{
	std::string sid;
	int2str(id,sid);
	return doc[LINESEGMENTS].remove(sid);
}
#endif /* MAP_SERVER_INCLUDE_MAP_SERVER_VIRTUAL_WALL_FUNCTION_HPP_ */
