/*
 * database_manager.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: tomzhang
 */
#include <ros/ros.h>
#include "map_server/database_manager.h"
#include "map_server/json_string_variables.h"
using namespace map_server;


//void Table::addFieldSql(std::string &sql)const
//{
//
//}

DataBaseManager::DataBaseManager():db_(NULL),is_ready_(false)
{

}
DataBaseManager::~DataBaseManager()
{
	closeDB();
}
bool DataBaseManager::openDB(std::string db_path)
{
	char *errMsg = 0;
	int rc =  sqlite3_open(db_path.c_str(),&db_);
	if(SQLITE_OK == rc)
	{
		is_ready_ = true;
		return true;
	}
	else
	{
		ROS_ERROR("open db failed!");
		sqlite3_free(errMsg);
		return false;
	}
}
bool DataBaseManager::closeDB()
{
    if (is_ready_)
    {
        int res = sqlite3_close(db_);
        if(res)
        {
            ROS_ERROR( "can't close database");
            return false;
        }
        is_ready_ = false;
        return true;
    }
    return false;
}
bool DataBaseManager::selectOperation(char* sql,std::vector<Region>& regions)
{
	char *errMsg = 0;
	
	int rc = sqlite3_exec(db_,sql,DataBaseManager::databaseCallback,&regions,&errMsg);
	if(SQLITE_OK == rc)
	{
		//ROS_ERROR(,sql);
		return true;
	}
	else
	{
		ROS_ERROR("sel %s",sql);
		ROS_ERROR("%s failed: %s\n",sql,errMsg);
		sqlite3_free(errMsg);
		return false;
	}

}
bool DataBaseManager::createTable(const char *sql)
{
	char *errMsg = 0;
	
	int rc = sqlite3_exec(db_,sql,0, 0,&errMsg);
	if(SQLITE_OK == rc)
	{
		return true;
	}
	else
	{
		ROS_DEBUG("cre s%s",sql);
		ROS_DEBUG("create table failed: %s\n", errMsg);
		sqlite3_free(errMsg);
		return false;
	}
}
//bool DataBaseManager::isTableFieldExist(const std::string &sql)const
//{
//	this->
//}
//bool DataBaseManager::isTableExist(const std::string &table_name) const
//{
//	if(table_name == NULL)
//	{
//		return false;
//		std::string sql = "SELECT COUNT(*) FROM SQLITE_MASTER WHERE TYPE = 'TABLE' AND NAME='" + //table_name+"'";
//		if( sqlite3_exec(db_, sql.c_str(), 0,0, &errMsg) != SQLITE_OK )
//		{
//		ROS_ERROR("insert value failed: %s\n", errMsg);
//		sqlite3_free(errMsg);
//		return false;
//	}
//	else
//	{
//		return true;
//	}
//	}
//
//}
bool DataBaseManager::insertValue(char* sql)
{
	char *errMsg = 0;
	
	if( sqlite3_exec(db_, sql, 0,0, &errMsg) != SQLITE_OK )
	{
		ROS_ERROR("ins %s",sql);
		ROS_ERROR("insert value failed: %s\n", errMsg);
		sqlite3_free(errMsg);
		return false;
	}
	else
	{
		return true;
	}
}
bool DataBaseManager::deleteValue(char* sql)
{
	char *errMsg = 0;

	if( sqlite3_exec(db_, sql, 0,0, &errMsg) != SQLITE_OK )
	{
        ROS_ERROR("del %s",sql);
		ROS_ERROR("delete value failed: %s\n", errMsg);
		sqlite3_free(errMsg);
		return false;
	}
	else
	{
		return true;
	}
}
bool DataBaseManager::updateValue(char* sql)
{
	char *errMsg = 0;
	
	if( sqlite3_exec(db_, sql, 0,0, &errMsg) != SQLITE_OK )
	{
		ROS_ERROR("upd %s",sql);
		ROS_ERROR("update value failed: %s\n", errMsg);
		sqlite3_free(errMsg);
		return false;
	}
	else
	{
		return true;
	}
}
//int DataBaseManager::checkTableCallback(void *NotUsed, int argc, char **argv, char **azColName)
//{
//	if(argc == 1)
//	{
//		int table_exist = atoi(*argv)
//	}
//	return 0;
//}
int DataBaseManager::databaseCallback(void *NotUsed, int argc, char **argv, char **azColName)
{
	//ROS_ERROR("in callback");
	std::vector<Region> *regions = (std::vector<Region>*)NotUsed;
	Region reg;
	reg.converFromSql(argv);
	regions->push_back(reg);
	return 0;
}
bool DataBaseManager::isReady()const{return is_ready_;}
