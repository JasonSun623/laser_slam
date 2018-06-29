#ifndef BASE_CMDVEL_MUX_H
#define BASE_CMDVEL_MUX_H

#include <libgen.h>
#include <fstream>
#include<vector>
#include <sys/stat.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "yaml-cpp/yaml.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

/*
*@class BaseCmdvelMuxCfg
*@brief save the items loaded from BaseCmdvelMuxCfg.yaml config file
*@name: velocity puber module name
*@priority: velocity puber module priority
*@state_time: velocity puber module keep control pri timeout
*/
class BaseCmdvelMuxCfg
{
	public:
		BaseCmdvelMuxCfg();
		~BaseCmdvelMuxCfg();

	public:
		std::string name;
		unsigned int priority;
		double state_time;
};

/*
*@class BaseCmdvelMuxPriList
*@brief local list to save modules property
*@name: velocity puber module name
*@priority: velocity puber module priority
*@state_time: velocity puber module keep control pri timeout
*@control_flag: control pri keeper flag, "1" hava right to set velocity to movebase
*@start_pub_time: pub velocity time
*/
class BaseCmdvelMuxPriList
{
	public:
		BaseCmdvelMuxPriList();
		~BaseCmdvelMuxPriList();

	public:
		std::string name;
		unsigned int priority;
		double state_time;
		char control_flag;
		ros::Time start_pub_time;
};

/*
*@class BaseCmdvelMuxClass
*@brief Mux cmdvel decide set or not to movebase
*/
class BaseCmdvelMuxClass
{
	public:
		BaseCmdvelMuxClass();
		~BaseCmdvelMuxClass();

		/* load velocity puber module config from  BaseCmdvelMuxCfg.yaml file */
		bool loadSystemPriCfg(void);
		/* handle velocity property when someone pub vel through topic */
		void handleSpeedCallback(const geometry_msgs::Twist::ConstPtr& speed,const BaseCmdvelMuxCfg* cfgp);
		/* create topics according cfg items */
		bool createBaseCmdvelMuxNode(void);
		/* create local priority list state module's property */
		bool createPriList(void);
		/* update base cmdvel state thread function */
		void update_base_cmdvel_mux_state(void);
		/* update local priority list state */
		void updateBaseCmdvelMuxPriList(BaseCmdvelMuxPriList* pl);
		/* clean vel timeout */
		void set_default_vel();

	public:
		ros::NodeHandle nh;
		ros::Publisher base_cmdvel_mux_pub;		//topic for pubging velocity to movebase
		BaseCmdvelMuxPriList speedPuber;		//speed Puber info
		std::string BaseCmdvelMuxCfgPath_;		//system cfg path
		std::vector<BaseCmdvelMuxCfg> systemPriCfg;		//local items to save property loaded from cfg file
		ros::V_Subscriber base_cmdvel_mux_sub;			//Subscriber register for every velocity puber module
		std::vector<BaseCmdvelMuxPriList> local_pri_list;	//local pri list for stating module's property
};

#endif
