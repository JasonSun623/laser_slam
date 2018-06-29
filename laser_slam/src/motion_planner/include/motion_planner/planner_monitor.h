/*
 * planner_monitor.h
 *
 *  Created on: Jan 30, 2018
 *      Author: tomzhang
 */

#ifndef INCLUDE_MOTION_PLANNER_PLANNER_MONITOR_H_
#define INCLUDE_MOTION_PLANNER_PLANNER_MONITOR_H_
#include <map>
#include <ros/ros.h>
namespace motion_planner
{
	class PlannerMonitor
	{
	public:
		PlannerMonitor();
		~PlannerMonitor();
		void AddPlannerClient(std::string &planner_name, ros::ServiceClient*);
		bool RecoveryMission(std::string &planner_name);
		void ClearMission();
		void CacheMission(std::string &cmd);
		void PopTop();
		void RemovePlannerClient(std::string &planner_name);



	private:
		std::map<std::string,ros::ServiceClient* > clients_;
		std::list<std::string> mission_stack_;
	};

}



#endif /* INCLUDE_MOTION_PLANNER_PLANNER_MONITOR_H_ */
