/*
 * planner_monitor.cpp
 *
 *  Created on: Jan 30, 2018
 *      Author: tomzhang
 */
#include "motion_planner/planner_monitor.h"
#include "motion_planner/MotionOp.h"
using namespace motion_planner;

PlannerMonitor::PlannerMonitor()
{

}
PlannerMonitor::~PlannerMonitor()
{

}
bool PlannerMonitor::RecoveryMission(std::string &planner_name)
{
    //ROS_ERROR("mission %s",planner_name.c_str());
	std::map<std::string,ros::ServiceClient* >::iterator it;
	if((it = this->clients_.find(planner_name))!=this->clients_.end())
	{
		if(it->second!=NULL)
		{
			for(std::list<std::string>::iterator iit = mission_stack_.begin(); iit != mission_stack_.end();++iit)
			{
				MotionOp srv;

				srv.request.cmd =*iit;
				//ROS_ERROR("%s ",(*iit).c_str());
				if(it->second->call(srv))
				{
					if(srv.response.error_code != 0)
					{
						ROS_ERROR("%s failed ",srv.request.cmd.c_str());
						return false;
					}
				}
				else
				{
					ROS_ERROR("%s not response ",planner_name.c_str());
					return false;
				}
			}
			return true;
		}
		else
		{
			ROS_ERROR("%s client point is NULL",planner_name.c_str());
		}
	}
	else
	{
		ROS_ERROR("%s not added",planner_name.c_str());
	}
	return false;
}
void PlannerMonitor::PopTop()
{
	mission_stack_.pop_back();
}
void PlannerMonitor::ClearMission()
{
	mission_stack_.clear();
}
void PlannerMonitor::CacheMission(std::string &cmd)
{
	mission_stack_.push_back(cmd);
}
void PlannerMonitor::AddPlannerClient(std::string &planner_name, ros::ServiceClient* client)
{
	ROS_ERROR("AddPlannerClient %s",planner_name.c_str());
	this->clients_[planner_name]=client;
}
void PlannerMonitor::RemovePlannerClient(std::string &planner_name)
{
	this->clients_.erase(planner_name);
}
