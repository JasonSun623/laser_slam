/*
 * CMDProcessor.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: tomzhang
 */


#include <ros/ros.h>
#include <std_msgs/String.h>
#include "map_server/cmd_processor.h"
#include "map_server/json_string_variables.h"


//constructor
CMDProcessor::CMDProcessor()
{
}
//destructor
CMDProcessor::~CMDProcessor()
{
	while(!task_queue_.empty())
		task_queue_.pop();
}
bool CMDProcessor::praserStringToJson(const std::string& cmd, Json::Value &root_in)const
{
	Json::Reader reader;
	if(!reader.parse(cmd, root_in))
	{
		ROS_ERROR("Invalid Json Message!");
		return false;
	}
	return true;
}

bool CMDProcessor::extractCMD(std::string &app_cmd, const Jptr root_in) const
{
	if ((*root_in)[PUB_NAME].isNull())
	{
		ROS_ERROR( "The json does not contain a pub_name tag or it is invalid.");
		return false;
	}
	//extract cmd
	app_cmd = (*root_in)[PUB_NAME].asString();
	return true;
}

bool CMDProcessor::empty()const
{
	return this->task_queue_.empty();
}

void CMDProcessor::popTask(Task &task)
{
	task = task_queue_.front();
	task_queue_.pop();
}
void CMDProcessor::pushTask(const Task task)
{
	task_queue_.push(task);
}




