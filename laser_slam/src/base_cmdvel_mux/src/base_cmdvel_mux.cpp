#include <stdio.h>
#include <stdlib.h>
#include "yaml-cpp/yaml.h"
#include "base_cmdvel_mux/base_cmdvel_mux.h"

#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif

static const unsigned int DEFAULT_STATE_TIME=1;
static const unsigned int DEFAULT_PRI_LEVEL=999;
static const std::string DEFAULT_PRI_NAME="default_pri_cmdvel";

BaseCmdvelMuxCfg::BaseCmdvelMuxCfg()
{
}
BaseCmdvelMuxCfg::~BaseCmdvelMuxCfg()
{
}

BaseCmdvelMuxPriList::BaseCmdvelMuxPriList()
{
}

BaseCmdvelMuxPriList::~BaseCmdvelMuxPriList()
{
}

BaseCmdvelMuxClass::BaseCmdvelMuxClass()
{
    ros::NodeHandle nh("base_cmdvel_mux");
    nh.param("BaseCmdvelMuxCfgPath", BaseCmdvelMuxCfgPath_, std::string(""));

	//load system priority config file
	loadSystemPriCfg();

	//create base cmdvel mux node
	createBaseCmdvelMuxNode();

	//create priority list, and scrible puber topic
	createPriList();

	ros::Rate rate(20);
	while(ros::ok())
	{
		update_base_cmdvel_mux_state();

		ros::spinOnce();
		rate.sleep();
	}
}

/*
* Loading system BaseCmdvelMux priority .yaml config file
*/
bool BaseCmdvelMuxClass::loadSystemPriCfg(void)
{
	int cfg_item = 0;
	BaseCmdvelMuxCfg localCfg;

	if(!BaseCmdvelMuxCfgPath_.compare(""))
	{
		ROS_DEBUG("[BaseCmdvelMuxClass]: NO BaseCmdvelMuxCfg file");
		return false;
	}

	std::ifstream fin(BaseCmdvelMuxCfgPath_.c_str());

	if(!fin){
		ROS_DEBUG("[BaseCmdvelMuxClass]: fin is NULL");
		return false;
	}else if(fin.fail()) {
		ROS_DEBUG("[BaseCmdvelMuxClass]: Open %s cfg file failed",BaseCmdvelMuxCfgPath_.c_str());
		return false;
	}else
		ROS_DEBUG("[BaseCmdvelMuxClass]: Open %s cfg file successed",BaseCmdvelMuxCfgPath_.c_str());

	//set default priority cfg
	localCfg.name = DEFAULT_PRI_NAME;
	localCfg.priority = DEFAULT_PRI_LEVEL;
	localCfg.state_time = DEFAULT_STATE_TIME;
	systemPriCfg.push_back(localCfg);

	speedPuber.name = DEFAULT_PRI_NAME;
	speedPuber.priority = DEFAULT_PRI_LEVEL;
	speedPuber.state_time = DEFAULT_STATE_TIME;
	speedPuber.start_pub_time = ros::Time::now();
	speedPuber.control_flag = 0;

	//load system priority cfgfile, save in systemPriCfg
	YAML::Node doc = YAML::Load(fin);

	cfg_item = doc.size();
	for(int i=0;i<cfg_item;i++)
	{
		doc[i]["name"] >> localCfg.name;
		doc[i]["priority"] >> localCfg.priority;
		doc[i]["state_time"] >> localCfg.state_time;

		systemPriCfg.push_back(localCfg);
	}

	fin.close();
	for(std::vector<BaseCmdvelMuxCfg>::iterator itp = systemPriCfg.begin();itp!=systemPriCfg.end();itp++)
		ROS_DEBUG("[BaseCmdvelMuxClass]: Load cfg[%s %d %.3f]",itp->name.c_str(),itp->priority,itp->state_time);

	return true;
}

/*
* Update local priority list after setting done
*/
void BaseCmdvelMuxClass::updateBaseCmdvelMuxPriList(BaseCmdvelMuxPriList* pl)
{
	if(!local_pri_list.empty())
	{
		for(std::vector<BaseCmdvelMuxPriList>::iterator itr = local_pri_list.begin();itr!=local_pri_list.end();itr++)
		{
			if(!speedPuber.name.compare(itr->name))
			{
				itr->priority = pl->priority;
				itr->control_flag = pl->control_flag;
				itr->start_pub_time = pl->start_pub_time;
			}
		}
	}
}

void BaseCmdvelMuxClass::set_default_vel()
{
	geometry_msgs::TwistStamped default_vel;

	default_vel.twist.linear.x = 0;
	default_vel.twist.linear.y = 0;
	default_vel.twist.linear.z = 0;
	default_vel.twist.angular.x = 0;
	default_vel.twist.angular.y = 0;
	default_vel.twist.angular.z = 0;
	default_vel.header.stamp = ros::Time::now();

	base_cmdvel_mux_pub.publish(default_vel);
	ROS_DEBUG("[BaseCmdvelMuxClass]: set default vel");
}

/*
* handle vel callback func according vel-puber priority
*/
void BaseCmdvelMuxClass::handleSpeedCallback(const geometry_msgs::Twist::ConstPtr& speed,const BaseCmdvelMuxCfg* cfgp)
{
	//for each prilist, find speed puber
	if(!systemPriCfg.empty())
	{
		for(std::vector<BaseCmdvelMuxCfg>::iterator itp = systemPriCfg.begin();itp!=systemPriCfg.end();itp++)
		{
			if(!cfgp->name.compare(itp->name))
			{
				speedPuber.name = itp->name;
				speedPuber.priority = itp->priority;
				speedPuber.state_time = itp->state_time;
			}
		}
	}

	//for each prilist, find pri_owner
	if(!local_pri_list.empty())
	{
		for(std::vector<BaseCmdvelMuxPriList>::iterator it = local_pri_list.begin();it!=local_pri_list.end();it++)
		{
			//confirm who have right to set velocity to movebase
			if(it->control_flag == 1)
			{
				ROS_DEBUG("[BaseCmdvelMuxClass]: BaseCmdvelMux Priority puber:%s, keeper:%s",speedPuber.name.c_str(),it->name.c_str());
				if(speedPuber.priority < it->priority)  //velocity puber's priority is higher than control_pri keeper's
				{
					it->control_flag = 0;
					speedPuber.control_flag = 1;
					speedPuber.start_pub_time = ros::Time::now();   //update state_time
					ROS_DEBUG("[BaseCmdvelMuxClass]: BaseCmdvelMux Priority keeper change <%s-%d> to <%s-%d>, Time(%.6f)",it->name.c_str(),it->priority,
					speedPuber.name.c_str(),speedPuber.priority,speedPuber.start_pub_time.toSec());

					updateBaseCmdvelMuxPriList(&speedPuber);
				}else if(speedPuber.priority == it->priority)   //velocity puber is the same
				{
					if(ros::Time::now() < (speedPuber.start_pub_time + ros::Duration(speedPuber.state_time)))   //velocity puber have time to keep control pri
					{
						ROS_DEBUG("[BaseCmdvelMuxClass]: Speed puber %s have time keep BaseCmdvelMux Priority, update time(%.6f)",
						speedPuber.name.c_str(),ros::Time::now().toSec());
						speedPuber.start_pub_time = ros::Time::now();   //update state_time

						updateBaseCmdvelMuxPriList(&speedPuber);
					}else   //velocity puber keep control pri timeout
					{
						ROS_DEBUG("[BaseCmdvelMuxClass]: Speed puber %s keep BaseCmdvelMux Priority timeout",speedPuber.name.c_str());
						//speed puber keep BaseCmdvelMux Priority timeout,set flag to "0"
						speedPuber.control_flag = 0;
						updateBaseCmdvelMuxPriList(&speedPuber);

						//set BaseCmdvelMux Priority keeper to default,keep one's control_flag is "1" in PriList at least
						speedPuber.name = DEFAULT_PRI_NAME;
						speedPuber.priority = DEFAULT_PRI_LEVEL;
						speedPuber.control_flag = 1;
						updateBaseCmdvelMuxPriList(&speedPuber);

						set_default_vel();
						return;
					}
				}else
				{
					ROS_DEBUG("[BaseCmdvelMuxClass]: The speed keepre pri > owner's, do nothing");
					return;
				}
			}
		}
	}
    
	//publish velocity to movebase
	geometry_msgs::TwistStamped speedStamped;
	speedStamped.twist = *speed;
	speedStamped.header.stamp = ros::Time::now();
	base_cmdvel_mux_pub.publish(speedStamped);
	ROS_DEBUG("[BaseCmdvelMuxClass]: %s pub Linear:[%.3f,%.3f,%.3f], Angular:[%.3f,%.3f,%.3f]",cfgp->name.c_str(),speed->linear.x,
	speed->linear.y,speed->linear.z,speed->angular.x,speed->angular.y,speed->angular.z);
	return;
}

/*
* create ros topic for every client
*/
bool BaseCmdvelMuxClass::createBaseCmdvelMuxNode(void)
{
	base_cmdvel_mux_pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

	ROS_DEBUG("[BaseCmdvelMuxClass]: create BaseCmdvelMuxNode::");
	if(!systemPriCfg.empty())
	{
		/*for(std::vector<BaseCmdvelMuxCfg>::iterator it=systemPriCfg.begin();it!=systemPriCfg.end();it++)
		{
		base_cmdvel_mux_sub.push_back(nh.subscribe<geometry_msgs::Twist>(it->name, 1000, boost::bind(&BaseCmdvelMuxClass::handleSpeedCallback, this, _1, it)));
		}*/
		/* register topics to the same callback func, and boost::bind the arg to it */
		for(int i = 0;i < systemPriCfg.size();i++)
		{
			base_cmdvel_mux_sub.push_back(nh.subscribe<geometry_msgs::Twist>(systemPriCfg[i].name, 1000, 
				boost::bind(&BaseCmdvelMuxClass::handleSpeedCallback, this, _1, &(systemPriCfg[i]))));
			ROS_DEBUG("[BaseCmdvelMuxClass]:    '%s'",systemPriCfg[i].name.c_str());
		}
	}

	return true;
}

/*
* Create local priority list for stating any vel-pub register
*/
bool BaseCmdvelMuxClass::createPriList(void)
{
	ROS_DEBUG("[BaseCmdvelMuxClass]: createPriList::");
	BaseCmdvelMuxPriList pl;

	for(std::vector<BaseCmdvelMuxCfg>::iterator it=systemPriCfg.begin();it!=systemPriCfg.end();it++)
	{
		ROS_DEBUG("[BaseCmdvelMuxClass]:    %s",it->name.c_str());
		pl.name = it->name;
		pl.priority = it->priority;
		pl.state_time = it->state_time;
		pl.start_pub_time = ros::Time::now();

		if(!it->name.compare(DEFAULT_PRI_NAME))
		{
			pl.control_flag = 1;    //set control pri to default  module
			set_default_vel();
		}else
			pl.control_flag = 0;

		local_pri_list.push_back(pl);
	}
	ROS_DEBUG("[BaseCmdvelMuxClass]: Init start pub time %.6f", ros::Time::now().toSec());

	for(std::vector<BaseCmdvelMuxPriList>::iterator itp=local_pri_list.begin();itp!=local_pri_list.end();itp++)
		ROS_DEBUG("[BaseCmdvelMuxClass]: To createPriList %s, control_flag %d",itp->name.c_str(), itp->control_flag);
}


/*
* Thread func for checking vel puber keep pri timeout
*/
void BaseCmdvelMuxClass::update_base_cmdvel_mux_state(void)
{
	if(!local_pri_list.empty())
	{
		for(std::vector<BaseCmdvelMuxPriList>::iterator it = local_pri_list.begin();it!=local_pri_list.end();it++)
		{
			/* cal control keeper's state time except default_pri_cmdvel */
			if((it->control_flag  == 1) && (it->name.compare(DEFAULT_PRI_NAME)))
			{
				if(ros::Time::now() >= (it->start_pub_time + ros::Duration(it->state_time)))
				{
					//speed puber keep BaseCmdvelMux Priority timeout,set flag to "0"
					it->control_flag = 0;

					//set BaseCmdvelMux Priority keeper to default
					for(std::vector<BaseCmdvelMuxPriList>::iterator itr = local_pri_list.begin();itr!=local_pri_list.end();itr++)
					{
						if(!itr->name.compare(DEFAULT_PRI_NAME))
						{
							ROS_DEBUG("[BaseCmdvelMuxClass]: Checking keep BaseCmdvelMux Priority timeout, set keeper to default");
							itr->control_flag = 1;

							set_default_vel();
						}
					}
				}
			}
		}
	}else
	{
		ROS_DEBUG("[BaseCmdvelMuxClass]: Error did not register anyone");
	}
}

BaseCmdvelMuxClass::~BaseCmdvelMuxClass()
{
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_cmdvel_mux");
	BaseCmdvelMuxClass BaseCmdvelMux;
	ros::spin();
	return 0;
}
