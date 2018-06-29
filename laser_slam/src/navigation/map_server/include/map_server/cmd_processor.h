/*
 * cmd_processor.h
 *
 *  Created on: Apr 1, 2017
 *      Author: tomzhang
 */

#ifndef MAP_SERVER_INCLUDE_MAP_SERVER_CMD_PROCESSOR_H_
#define MAP_SERVER_INCLUDE_MAP_SERVER_CMD_PROCESSOR_H_


#include <map>
#include <queue>
#include "json/json.h"
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
typedef boost::function<void(boost::shared_ptr<Json::Value>,Json::Value&, int&,std::string&)> Func;
typedef std::map<std::string,Func> FuncMap;
typedef boost::shared_ptr<FuncMap> FuncMapPtr;
typedef boost::shared_ptr<Json::Value> Jptr;
typedef boost::function<void (void) > Task;

class CMDProcessor
{
public:
	CMDProcessor();
	~CMDProcessor();
	bool praserStringToJson(const std::string& cmd, Json::Value &root_in)const;
	bool extractCMD(std::string &app_cmd, const Jptr root_in)const;
	void popTask(Task &task);
	void pushTask(const Task task);
	bool empty()const;

private:
	std::queue<Task> task_queue_;
};

#endif /* MAP_SERVER_INCLUDE_MAP_SERVER_CMD_PROCESSOR_H_ */
