/*
 * database_manager.h
 *
 *  Created on: Aug 30, 2017
 *      Author: tomzhang
 */

#ifndef INCLUDE_MAP_SERVER_DATABASE_MANAGER_H_
#define INCLUDE_MAP_SERVER_DATABASE_MANAGER_H_
#include <sqlite3.h>
#include "map_server/region.h"

namespace map_server
{

class DataBaseManager
{
public:
	DataBaseManager();
	~DataBaseManager();
	bool openDB(std::string db_path);
	bool closeDB();
	bool selectOperation(char* sql,std::vector<Region> &regions);
	bool createTable(const char *sql);
	//bool isTableExist(const std::string &table_name) const;
	//bool createTableField(const char *sql);
	bool isTableFieldExist(const std::string &sql) const;
	bool insertValue(char* sql);
	bool deleteValue(char* sql);
	bool updateValue(char* sql);

	bool isReady() const;
	static int databaseCallback(void *NotUsed, int argc, char **argv, char **azColName);
private:

	sqlite3 *db_;
	bool is_ready_;
};
}



#endif /* INCLUDE_MAP_SERVER_DATABASE_MANAGER_H_ */
