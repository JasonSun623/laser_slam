#include <boost/algorithm/string.hpp>
 
#include "localization/localization.h"

static const char* STATE_OK = "ok";
static const char* SPLIT_CHAR = ",";

// localization yaml item
static const char* MODULE = "module";
static const char* NAV_MODULE = "nav_module";
static const char* MAP_MODULE = "map_module";
static const char* DEFAULT_MODULE = "default_module";
static const char* MAP_USE = "map_use";
static const char* NAV_USE = "nav_use";

LocalizationModule::~LocalizationModule()
{
    LocalizationModule::module_map.clear();
}

/*
* set module name and state
*/
void LocalizationModule::setModuleState(std::string name,std::string state)
{
    LocalizationModule::module_map.insert(
        std::map<std::string,std::string>::value_type(name, state));
}

/*
* set module ok
*/
void LocalizationModule::registerModule(std::string name)
{
    LocalizationModule::setModuleState(name,STATE_OK);
}

std::vector<std::string> LocalizationModule::queryModule()
{
    std::vector<std::string> vModule;
    std::string moduleStr = lp.getParam(MODULE);

     //get module vector spilt modules string in yaml
     //boost::token_compress_on means many series splitchar hand to one splitchar
    boost::split(vModule, moduleStr, boost::is_any_of(SPLIT_CHAR), boost::token_compress_on);

    return vModule;
}

/*
* is all module ok from modules in yaml and module map
*/
bool LocalizationModule::isModulesOK()
{
    bool rlt = false;
    std::vector<std::string> vModule = queryModule();
    std::map<std::string,std::string>::iterator ptr;

    for(std::vector<std::string>::iterator it = vModule.begin(); it != vModule.end(); ++ it)
    {
       //get module state from module_map
       ptr = LocalizationModule::module_map.find(*it);
       //ROS_INFO("module %s,state %s",(*it).c_str(),(*ptr).second.c_str());
       // if all module is ok return ok ,else have one false return false
       if((ptr != module_map.end()) && (0 == (*ptr).second.compare(STATE_OK)))
       {
            rlt = true;
       }
       else 
       {
            rlt = false;
            break;
       }
    }
    return rlt;
}

/*
* query nav module from yaml
*/
std::string LocalizationModule::queryNavModule()
{
    return lp.getParam(NAV_MODULE);
}

/*
* query map module from yaml
*/
std::string LocalizationModule::queryMapModule()
{
    return lp.getParam(MAP_MODULE);
}

/*
* get current nav module from yaml
*/
std::string LocalizationModule::getCurrentNavModule()
{
    return lp.getParam(NAV_USE);
}

/*
* get current map module from yaml
*/
std::string LocalizationModule::getCurrentMapModule()
{
    return lp.getParam(MAP_USE);
}

/*
* get default modult from yaml
*/
std::string LocalizationModule::getDefaultModule()
{
    return lp.getParam(DEFAULT_MODULE);
}

/*
* set current nav module to yaml
*/
int LocalizationModule::setCurrentNavModule(std::string navCurrent)
{
    return lp.setParam(NAV_USE,navCurrent);
}

/*
* set current map module to yaml
*/    
int LocalizationModule::setCurrentMapModule(std::string mapCurrent)
{
    return lp.setParam(MAP_USE,mapCurrent);
}

/*
* set default module to yaml
*/
int LocalizationModule::setDefaultModule(std::string defaultModule)
{
    return lp.setParam(DEFAULT_MODULE,defaultModule);
}

/*
* get amcl param manager
*/
SubParamManager LocalizationModule::getAmclParamManager()
{
    return SubParamManager(lp.amcl_attribute_yaml_path,lp.amcl_yaml_path);
}
        
/*
* get gmapping param manager
*/
SubParamManager LocalizationModule::getGmappingParamManager()
{
    return SubParamManager(lp.gmapping_attribute_yaml_path,lp.gmapping_yaml_path);
}

/*
* get uwb param manager
*/
SubParamManager LocalizationModule::getUwbParamManager()
{
    return SubParamManager(lp.uwb_attribute_yaml_path,lp.uwb_yaml_path);
}