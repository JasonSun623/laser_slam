/*
 * region_manager.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: tomzhang
 */

#include "map_server/region_manager.h"
#include "map_server/json_string_variables.h"

using namespace map_server;
RegionManager::RegionManager(std::string root_path):dbmgr_(NULL),pnh_("~/region_params_changer")
{
	std::string odom_topic;
	nh_.param("odom_topic",odom_topic, std::string("/current_pose"));
	nh_.param("param_check_rate",param_check_rate_,5);
	initTables();
	dbmgr_ = new DataBaseManager();
	odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 1, boost::bind(&RegionManager::odomCallback, this, _1));

	params_service_ = pnh_.advertiseService("param_update",&RegionManager::params_service,this);
	fixpath_params_updater_ = nh_.serviceClient<motion_planner::MotionOp>("/move_base/MrobotPlannerROS/fixpath_params_update");
	//map_list_sub_ = nh_.subscribe("/map_list_pub", 10,&RegionManager::jsonCallback, this);
	this->audio_params_pub_ =  pnh_.advertise<std_msgs::String>("audio_params", 1, true);
	this->sensor_params_pub_ = pnh_.advertise<std_msgs::String>("sensor_params", 1, true);;
	this->path_ = root_path;
	param_pub_thread_ = boost::thread(boost::bind(&RegionManager::param_changer_fun,this));
	param_pub_thread_.detach();

}
RegionManager::~RegionManager()
{
	if(dbmgr_!=NULL)
		delete(dbmgr_);
}
bool RegionManager::params_service(motion_planner::MotionOp::Request  &req,
		motion_planner::MotionOp::Response &res)
{
	ROS_ERROR("%s request parameters service update params!",req.cmd.c_str());
	update();
	res.error_code = 0;
	res.result = "Params Updated Success!";
	return true;


}
bool RegionManager::robotPosCheck(const std::vector<Vertex> &vts, const geometry_msgs::Point &pos)const
{
	int i,j=vts.size()-1 ;
	bool  oddNodes= false;

	for (i=0;i<vts.size(); i++)
	{

		if(((vts[i].y< pos.y && vts[j].y>=pos.y)|| (vts[j].y<pos.y &&vts[i].y>=pos.y))&& (vts[i].x<=pos.x || vts[j].x<=pos.x))
		{
			oddNodes^=(vts[i].x+(pos.y-vts[i].y)/(vts[j].y-vts[i].y)*(vts[j].x-vts[i].x)<pos.x);
		}
		j=i;
	}
	return oddNodes;
}
void RegionManager::param_changer_fun()
{
	ros::Rate r(param_check_rate_);
	while(!db_loaded_)
	{
		boost::mutex::scoped_lock lock(db_loaded_mtx_);
		db_loaded_cond_.wait(db_loaded_mtx_);
	}
	std::vector<std::vector<Region> > regions_list;
	Region nav_finded_region,loc_finded_region,audio_finded_region,sensor_finded_region,default_region;
	std::vector<Region> finded_regions;
	nav_finded_region.region_id = -1;
	loc_finded_region.region_id = -1;
	audio_finded_region.region_id = -1;
	sensor_finded_region.region_id = -1;

	//bool change_region = false;//keep_region =false;
	for(int type  = 1; type <=4; ++type)
	{
		ROS_INFO("init regions list! region type %d",type);
		std::vector<Region> regions;
		getRegionsByType(type,regions);
		regions_list.push_back(regions);
		Region finded_region;
		finded_region.region_id = -1;
		finded_region.region_type = type;
		finded_regions.push_back(finded_region);
		
	}

	//rmgr->getRegionsByMap(scene_name_,map_name_, regions);
	getRegionByID(0,default_region);
	geometry_msgs::Point pos;

	while(ros::ok)
	{
		{
			boost::mutex::scoped_lock lock(odom_lock_);
			pos  = base_odom_.pose.pose.position;
		}
		{
			boost::mutex::scoped_lock lock(update_mtx_);
			if(region_updated_)
			{
				ROS_INFO("regions updated");
				regions_list.clear();
				for(int type  = 1; type <=4; ++type)
				{
					std::vector<Region> regions;
					getRegionsByType(type,regions);
					regions_list.push_back(regions);
					finded_regions[type-1].region_id = -1;
				}
				getRegionByID(0,default_region);
				region_updated_ = false;
			}
		}
		for(int i = 0; i < 4;++i )
		{
			//ROS_ERROR("check regions type %d",i+1);
			regionCheck(i+1,finded_regions[i],default_region,regions_list[i],pos);
		}

		r.sleep();
	}
}
void RegionManager::regionCheck(int region_type, Region &finded_region,const Region &default_region,const std::vector<Region>&regions,const geometry_msgs::Point &pos)
{
	bool change_region = false;
	Region in_region = default_region;
	for(int i = 0; i < regions.size();++i)
	{
		if(robotPosCheck(regions[i].vts, pos))
		{
			if(regions[i].param.priority>=in_region.param.priority)
			{
				in_region = regions[i];
			}
		}
	}
	if(finded_region.region_id != in_region.region_id)
	{
		finded_region = in_region;
		change_region = true;
		//ROS_ERROR("find region id %d", finded_region.region_id );
	}
	if(change_region)
	{
		switch (region_type)
		{
		case 1:navParamsPub(finded_region);break;
		case 2:locParamsPub(finded_region);break;
		case 3:audioParamsPub(finded_region);break;
		case 4:sensorParamsPub(finded_region);break;
		default:;

		};
	}
}
void RegionManager::navParamsPub(Region &finded_region)
{
		Json::Value jpars,jdata;
		finded_region.param.convertToNavJson(jdata);
		jpars[PARAMS]=jdata;
		Json::StyledWriter fast;
		motion_planner::MotionOp srv;
		srv.request.cmd = fast.write(jpars);
		if(!fixpath_params_updater_.call(srv))
		{
			finded_region.region_id = -1;
			ROS_INFO("send nav param failed!");
			return;
		}
		ROS_INFO("send nav param %s",srv.request.cmd.c_str());
}
void RegionManager::locParamsPub(const Region &finded_region)
{
	//ROS_INFO("send amcl param");
	ros::param::set("/snap_map_icp/icp_inlier_threshold",finded_region.param.threshold);
	ros::param::set("/snap_map_icp/snapmapicp_open",finded_region.param.enable_map_registeration);
	ros::param::set("/snap_map_icp/camera_set_pose_valid_durtime",finded_region.param.mark_duration);
	ROS_INFO("set loc params icp_inlier_threshold:%f,snapmapicp_open:%d,camera_set_pose_valid_durtime:%f",finded_region.param.threshold,finded_region.param.enable_map_registeration,finded_region.param.mark_duration);
}
void RegionManager::audioParamsPub(const Region &finded_region)
{
	//ROS_ERROR("send audio param");
	Json::Value jpars,jdata;
	finded_region.param.convertToAudioJson(jdata);
	jpars[PARAMS]=jdata;
	Json::StyledWriter fast;
	std_msgs::String params_str;
	params_str.data= fast.write(jpars);
	ROS_INFO("send audio param %s",params_str.data.c_str());
	audio_params_pub_.publish(params_str);
}
void RegionManager::sensorParamsPub(const Region &finded_region)
{
    //ROS_ERROR("send sensor param");
	Json::Value jpars,jdata;
	finded_region.param.convertToSensorJson(jdata);
	jpars[PARAMS]=jdata;
	Json::StyledWriter fast;
	std_msgs::String params_str;
	params_str.data= fast.write(jpars);
	ROS_INFO("send sensor param %s",params_str.data.c_str());
	sensor_params_pub_.publish(params_str);

}
void RegionManager::odomCallback(const nav_msgs::Odometry::ConstPtr msg )
{
	{
		boost::mutex::scoped_lock lock(odom_lock_);
		base_odom_.pose.pose= msg->pose.pose;
	}
}

void RegionManager::checkAndSynDB()
{

	for(std::vector<Table>::iterator table_it = tables_.begin();table_it!= tables_.end();++table_it)
	{
		std::string sql;
		
		table_it->createTableSql(sql);
		if(!this->dbmgr_->createTable(sql.c_str()))
		{// the table exists, we do nothing TODO: we check fields one by one and insert fields which not exist in the table, and also set default values.
			ROS_INFO("table %s exist",table_it->table_name.c_str());
//			for(std::vector<Field>::iterator field_it = (table_it->fields).begin(); field_it != (table_it->fields).end(); ++field_it)
//			{
//				if()
//				sql.clear();
//				sql.append("ALTER TABLE ");
//				sql.append(table_it->table_name);
//				sql.append(" ADD COLUMN ");
//				field_it->createFieldSql(sql);
//				this->dbmgr_->createTableField(sql.c_str());
//			}
			int id = 0;
			getMaxRegionID(id);
			if(id<0)
			{
				addDefaultRegion();
			}
		}
		else 
		{// the table do not exists, we created it and set default value;
			addDefaultRegion();
		}
	}

}
void RegionManager::addDefaultRegion()
{
	Region region;
	region.region_name = "default_region";
	region.region_type = 0;
	region.region_id = 0;
	Vertex ver;
	ver.id = 0;
	ver.x =0.0;
	ver.y = 0.0;
	Vertex ver1;
	ver1.id = 1;
	ver1.x =0.0;
	ver1.y = 0.0;
	Vertex ver2;
	ver2.id = 2;
	ver2.x =0.0;
	ver2.y = 0.0;
	Vertex ver3;
	ver3.id = 3;
	ver3.x =0.0;
	ver3.y = 0.0;
	region.vts.push_back(ver);
	region.vts.push_back(ver1);
	region.vts.push_back(ver2);
	region.vts.push_back(ver3);
	region.param.max_vel_x = 0.5;
	region.param.acc_x = 0.8;
	region.param.max_th =0.5;
	region.param.acc_th = 1.0;
	region.param.collision_stop_dist = 0.5;
	region.param.collision_avoidance = 0;
	region.param.path_tolorance =0.5;
	region.param.nav_type = 1;
	region.param.audio_name = "no name";
	region.param.interval_time = 1.0;
	region.param.enable_map_registeration = 1;
	region.param.threshold = 0.9;
	region.param.mark_duration = 5.0;
	region.param.enable_supersonic = 2147483647;
	region.param.enable_microlaser = 8191;
	region.param.enable_camera = 15;
	region.param.enable_imu = 1;
	region.param.enable = 1;
	region.param.priority = 0;
	region.param.waiting_time = 10.0;
	createRegion(region);
}


int RegionManager::openRegionsBD(std::string scene_name, std::string map_name)
{
	if(this->dbmgr_->isReady())
	{
		this->dbmgr_->closeDB();
	}
	if(this->dbmgr_->openDB(path_ +scene_name+"/regions/" + map_name+".db"))
	{
		checkAndSynDB();
		update();
		boost::mutex::scoped_lock lock(db_loaded_mtx_);
		db_loaded_ = true;
		db_loaded_cond_.notify_all();
	}
	return 0;
}
void RegionManager::initTables()
{
	Table region_table;
	region_table.table_name = REGIONS;
	region_table.fields.push_back(Field(REGION_ID,1));
	region_table.fields.push_back(Field(REGION_NAME,2));
	region_table.fields.push_back(Field(REGION_TYPE,1));
	region_table.fields.push_back(Field(VERTEX,2));
	region_table.fields.push_back(Field(MAX_VEL_X,0));
	region_table.fields.push_back(Field(ACC_X,0));
	region_table.fields.push_back(Field(MAX_TH,0));
	region_table.fields.push_back(Field(ACC_TH,0));
	region_table.fields.push_back(Field(COLLISION_STOP_DIST,0));
	region_table.fields.push_back(Field(COLLISION_AVOIDANCE,1));
	region_table.fields.push_back(Field(WAITING_TIME,0));
	region_table.fields.push_back(Field(PATH_TOLORANCE,0));
	region_table.fields.push_back(Field(NAV_TYPE,1));
	region_table.fields.push_back(Field(AUDIO_NAME,2));
	region_table.fields.push_back(Field(INTERVAL_TIME,0));
	region_table.fields.push_back(Field(THRESHOLD,0));
	region_table.fields.push_back(Field(ENABLE_MAP_REGISTERATION,1));
	region_table.fields.push_back(Field(MARK_DURATION,1));
	region_table.fields.push_back(Field(ENABLE_SUPERSONIC,1));
	region_table.fields.push_back(Field(EBALBE_MICROLASER,1));
	region_table.fields.push_back(Field(ENABLE_CAMERA,1));
	region_table.fields.push_back(Field(ENABLE_IMU,1));
	region_table.fields.push_back(Field(ENABLE,1));
	region_table.fields.push_back(Field(PRIORITY,1));
	tables_.push_back(region_table);

}
void RegionManager::update()
{
	boost::mutex::scoped_lock lock(update_mtx_);
	this->region_updated_ = true;

}
//bool RegionManager::findRegion(const std::vector<Region> &regions,const geometry_msgs::Point &cur_pos, Region &region)
//{
//	Region in_region = default_region;
//	for(int i = 0; i < regions.size();++i)
//	{
//		if(regionCheck(regions[i].vts, cur_pos))
//		{
//			if(regions[i].param.priority>=region.param.priority)
//			{
//                in_region = regions[i];
//			}
////				keep_region = true;
//		}
//	}
//    if(finded_region.region_id != region.region_id)
//    {
//        finded_region = in_region;
//        change_region = true;
//    }
//}
int RegionManager::createRegion(Json::Value &data_in)
{

	int err = 0;
	int id;
	Region region;
	if(0!=(err =region.convertFromJson(data_in)))
	{
		return err;
	}

	if(0==(err=createRegion(region)))
	{
		data_in[REGION_ID] = region.region_id;
		update();
	}
	return err;
}
int RegionManager::createRegion(Region &region)
{

	//int err = 0;
	int id;
	getMaxRegionID(id);
	if(id == 0)
		region.region_id =id+100;
	else if(id == -1)
	{
		ROS_INFO("no regions");
	}
	else
	{
		region.region_id = ++id;
	}

	std::string sql_str;
	region.convertToInsertSql(sql_str);
	char sql[1024];
	std::sprintf(sql,"%s VALUES (%s);",INSERT.c_str(),sql_str.c_str());
	//ROS_ERROR("create %s",sql);
	if(!dbmgr_->insertValue(sql))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
int RegionManager::createRegionID()
{
	return 0;
}
int RegionManager::getRegionsByNav(int nav_type,Json::Value &data_in)
{
	return 0;
}
void RegionManager::getRegionsByType(int region_type, std::vector<Region> &regions)
{
	char sql[1024];
	std::sprintf(sql,"SELECT * FROM %s WHERE %s =%d",REGIONS.c_str(),REGION_TYPE.c_str(),region_type);
	//ROS_ERROR("get regions by id %s",sql);
	if(dbmgr_->selectOperation(sql,regions))
	{
		if(regions.empty())
		{
			ROS_ERROR("region_type:%d empty",region_type);
		}
	}
}
int RegionManager::getRegionsByMap(Json::Value &data_in)
{

	std::vector<Region> regions;
	int err;
	if(0 ==(err = getRegionsByMap(regions)))
	{
		Json::Value data_out;
		for(int i = 0 ; i<regions.size();++i)
		{
			Json::Value reg;
			regions[i].convertToJson(reg);
			data_out[REGIONS].append(reg);
		}
		data_in = data_out;
	}
	return err;
}
int RegionManager::getRegionsByMap(std::vector<Region> &regions)
{
	char sql[1024];
	std::sprintf(sql,"SELECT * FROM %s",REGIONS.c_str());

	if(dbmgr_->selectOperation(sql,regions))
	{
		if(regions.empty())
		{
			return 2;
		}
		return 0;
	}
	else
	{
		return 1;
	}

}
int RegionManager::getRegionByID(int region_id, Json::Value &data_in)
{

	Region region;
	int err = 0;
	if(0 ==(err = getRegionByID(region_id, region)))
	{
		region.convertToJson(data_in);
	}
	return err;
}
int RegionManager::getRegionByID(int region_id, Region &region)
{
	char sql[1024];
	std::sprintf(sql,"SELECT * FROM %s WHERE %s =%d",REGIONS.c_str(),REGION_ID.c_str(),region_id);
	//ROS_ERROR("get regions by id %s",sql);
	std::vector<Region> regions;
	if(dbmgr_->selectOperation(sql,regions))
	{
		if(regions.empty())
		{
			return 2;
		}
		region = regions.front();
		return 0;
	}
	else
	{
		return 1;
	}
}
void RegionManager::getMaxRegionID(int& region_id)
{
	char sql[1024];
	std::sprintf(sql,"SELECT * FROM %s WHERE %s=(SELECT MAX(%s) FROM %s)",REGIONS.c_str(),REGION_ID.c_str(),REGION_ID.c_str(),REGIONS.c_str());
	//ROS_ERROR("max id %s",sql);
	std::vector<Region> regions;
	dbmgr_->selectOperation(sql,regions);
	if(regions.empty())
		region_id = -1;
	else
		region_id =  regions[0].region_id;
}
int RegionManager::removeRegionByID(int region_id)
{
	char sql[1024];
	std::sprintf(sql,"DELETE FROM %s WHERE %s = %d",REGIONS.c_str(),REGION_ID.c_str(),region_id);
	if(dbmgr_->deleteValue(sql))
	{
		update();
		return 0;
	}
	else
	{
		return 1;
	}

}
int RegionManager::modifyRegion(Json::Value &data_in)
{
	std::string reg;
	int err = 0;
	Region region;
	if(0!=(err =region.convertFromJson(data_in)))
	{
		return err;
	}
	region.convertToUpdateSql(reg);
	char sql[2048];
	std::sprintf(sql,"UPDATE %s SET %s WHERE %s =%d",REGIONS.c_str(),reg.c_str(),REGION_ID.c_str(), region.region_id);
	//ROS_ERROR("%s",sql);
	if(dbmgr_->updateValue(sql))
	{
		update();
		return 0;
	}
	else
	{
		return 1;
	}
}

