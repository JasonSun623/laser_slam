#include "localization/localization_sub_params.h"

static const std::string NAME = "name";
static const std::string TYPE = "type";
static const std::string MAX = "max";
static const std::string MIN = "min";
static const std::string DEFAULT = "default";
static const std::string INT = "int";
static const std::string BOOL = "bool";
static const std::string DOUBLE = "double";
static const std::string STRING = "string";
static const std::string SPLIT_CHAR = "|";

SubParamManager::SubParamManager(std::string paramAttiYaml,std::string paramYaml)
{
    this->paramAttiYaml = paramAttiYaml;
    this->paramYaml = paramYaml;
    // get sub module param attribute list from yaml in init
    getParamAttiVector();
    // get sub module param list from yaml in init
    getParamValueVector();
}

/*
* app get sub module param list
*/
std::vector<SubParamValue> SubParamManager::getSubParamsVector()
{
    return subParamsValueVector;
}

/*
* app set sub module param
*/
int SubParamManager::setSubParamValue(std::string name, Json::Value jsonValue)
{
    boost::any value;
    int rlt = checkValue(name,jsonValue,value);
    
    if(SET_SUCCESS != rlt)
    {
        return rlt;
    }
    std::ifstream fin(paramYaml.c_str());
    
    // open fin fail
    if(!fin)
    {
        ROS_ERROR("fail open sub param yaml");
        return OPEN_ERROR;
    }
    
    YAML::Node doc = YAML::Load(fin);
    fin.close();
    std::ofstream fout(paramYaml.c_str());
    // open fout fail
    if(!fout)
    {
        ROS_ERROR("fail open sub param yaml");
        return OPEN_ERROR;
    }

    SubParamAttribute subParamAttribute = getParamAtti(name);
    //set doc for different type
    try
    {
        if(0 == subParamAttribute.type.compare(INT))
        {
            doc[name] = boost::any_cast<int>(value);
        }
        else if (0 == subParamAttribute.type.compare(DOUBLE))
        {
            doc[name] = boost::any_cast<double>(value); 
        }
        else if (0 == subParamAttribute.type.compare(BOOL))
        {
            doc[name] = boost::any_cast<bool>(value);
        }
        else if (0 == subParamAttribute.type.compare(STRING))
        {
            doc[name] = boost::any_cast<std::string>(value);
        }
        fout << doc; 
    }
    catch(YAML::Exception)
    {
       ROS_ERROR("LocalizationParameter set invalid tag");
       rlt = SET_ERROR;
    }
    fout.close();
    return rlt;
}

/*
*  check set value is right
*/
int SubParamManager::checkValue(std::string name,Json::Value jsonValue, boost::any& value)
{
    int rlt = SET_SUCCESS;
    //check empty
    if(jsonValue.isNull())
    {
        rlt = EMPTY_ERROR;
    }
    // check and trans type
    else if(TYPE_ERROR == transParamType(name,jsonValue,value))
    {
        rlt = TYPE_ERROR;
    }
    //check range
    else if(!isParamInRange(name,value))
    {
        rlt = RANGE_ERROR;
    }
    return  rlt;
}

/*
* trans json value into real type
*/
int SubParamManager::transParamType(std::string name, Json::Value jsonValue, boost::any& value)
{
    SubParamAttribute subParamAttribute = getParamAtti(name); 
    std::string paramType = subParamAttribute.type;
    int rlt = SET_SUCCESS;
    if(0 == paramType.compare(INT))
    {
        if(jsonValue.isInt())
        {
            value = jsonValue.asInt();
        }
        else
        {
            rlt = TYPE_ERROR;
        }
    }
    else if (0 == paramType.compare(BOOL))
    {
        if(jsonValue.isBool())
        {
            value = jsonValue.asBool();
        }
        else
        {
            rlt = TYPE_ERROR;
        }
    }
    else if (0 == paramType.compare(DOUBLE))
    {
        if(jsonValue.isDouble())
        {
            value = jsonValue.asDouble();
        }
        else
        {
            rlt = TYPE_ERROR;
        }
    }
    else if (0 == paramType.compare(STRING))
    {
        if(jsonValue.isString())
        {
            value = jsonValue.asString();
        }
        else
        {
            rlt = TYPE_ERROR;
        }
    }
    return rlt;
}

/*
* is set value is in range
*/
bool SubParamManager::isParamInRange(std::string name, boost::any value)
{
    bool rlt = false;
    SubParamAttribute subParamAttribute = getParamAtti(name); 
    std::string paramType = subParamAttribute.type;
    std::vector<std::string> vModule;
    std::string maxString;
    // int: min <= value <= max
    if(0 == paramType.compare(INT))
    {
        rlt = (boost::any_cast<int>(value) >= boost::any_cast<int>(subParamAttribute.min)
            && boost::any_cast<int>(value) <= boost::any_cast<int>(subParamAttribute.max));
    }
    // double: min <= value <= max
    else if(0 == paramType.compare(DOUBLE))
    {
        rlt = (boost::any_cast<double>(value) >= boost::any_cast<double>(subParamAttribute.min)
            && boost::any_cast<double>(value) <= boost::any_cast<double>(subParamAttribute.max));
    }
    // bool: value = true || value = false
    else if(0 == paramType.compare(BOOL))
    {
        rlt = (boost::any_cast<bool>(value) == true || boost::any_cast<bool>(value) == false);
    }
    // string : value in max(A|B|...)
    else if(0 == paramType.compare(STRING))
    {
        maxString = boost::any_cast<std::string>(subParamAttribute.max);
        if(0 == maxString.compare(""))
        {
            rlt = true;
        }
        //split max by |
        else
        {
            boost::split(vModule, maxString,boost::is_any_of(SPLIT_CHAR), boost::token_compress_on);
            for(std::vector<std::string>::iterator it = vModule.begin(); it != vModule.end(); ++ it)
            {
                // is string value in range
                if (0 == (boost::any_cast<std::string>(value)).compare(*it))
                {
                    rlt = true;
                    break;
                }
            }
        }
    }
    return rlt;
}

/*
*  get param vaule for name in param yaml
*/
boost::any SubParamManager::getParamValue(std::string name)
{
    boost::any rlt;
    std::ifstream fin(paramYaml.c_str());

    // open fail
    if(!fin)
    {
        ROS_ERROR("fail open localization sub param yaml");
        return rlt;
    }
    YAML::Node doc = YAML::Load(fin);

    SubParamAttribute subParamAttribute = getParamAtti(name);
    //handle value type for doc
    try
    {
        if(0 == subParamAttribute.type.compare(INT))
        {
            rlt = doc[name].as<int>(); 
        }
        else if (0 == subParamAttribute.type.compare(DOUBLE))
        {
            rlt = doc[name].as<double>(); 
        }
        else if (0 == subParamAttribute.type.compare(BOOL))
        {
            rlt = doc[name].as<bool>();
        }
        else if (0 == subParamAttribute.type.compare(STRING))
        {
            rlt = doc[name].as<std::string>();
        }
    }
    catch(YAML::Exception)
    {
       ROS_ERROR("LocalizationParameter get invalid tag");
    }
    fin.close();
    return rlt;
}

/*
* get sub module param list in init
* vector e.g. name:xx type:xx, value:xx
*/
void SubParamManager::getParamValueVector()
{
    SubParamValue subParamValue;
    for(std::vector<SubParamAttribute>::iterator it = subParamsAttiVector.begin(); it != subParamsAttiVector.end(); ++ it)
    {
        subParamValue.name = (*it).name;
        subParamValue.type = (*it).type;
        subParamValue.value = getParamValue(subParamValue.name);

        subParamsValueVector.push_back(subParamValue);
    }
}

/*
* get param attribute for name
*/
SubParamAttribute SubParamManager::getParamAtti(std::string name)
{
    SubParamAttribute subParamAttribute;
    for(std::vector<SubParamAttribute>::iterator it = subParamsAttiVector.begin(); it != subParamsAttiVector.end(); ++ it)
    {
        //compate name and attibute vector name
        if(0 == (*it).name.compare(name))
        {
            subParamAttribute = *it;
            break;
        }
    }
    return subParamAttribute;
}

/*
* get sub module param attribute list from yaml in init
*/
void SubParamManager::getParamAttiVector()
{
     std::ifstream fin(paramAttiYaml.c_str());
     // open fail
     if(!fin)
     {
         ROS_ERROR("fail open localization sub param yaml");
         return;
     }
     YAML::Node doc = YAML::Load(fin);

     SubParamAttribute subParamAttribute;
     int itmeSize = doc.size();

     // handle param attibute yaml 
     for(int i=0; i<itmeSize; i++)
     {
         try
         {
             subParamAttribute.name = doc[i][NAME].as<std::string>();
             subParamAttribute.type = doc[i][TYPE].as<std::string>();
             // int : max min default
             if(0 == subParamAttribute.type.compare(INT)){
                 subParamAttribute.max = doc[i][MAX].as<int>();
                 subParamAttribute.min = doc[i][MIN].as<int>();
                 subParamAttribute.defaultValue = doc[i][DEFAULT].as<int>();
             }
             //double: max min default
             else if (0 == subParamAttribute.type.compare(DOUBLE))
             {
                 subParamAttribute.max = doc[i][MAX].as<double>();
                 subParamAttribute.min = doc[i][MIN].as<double>();
                 subParamAttribute.defaultValue = doc[i][DEFAULT].as<double>();
             }
             //bool: default
             else if (0 == subParamAttribute.type.compare(BOOL))
             {
                 subParamAttribute.defaultValue = doc[i][DEFAULT].as<bool>();
             }
             //string: max(A|B|..) deault
             else if (0 == subParamAttribute.type.compare(STRING))
             {
                 subParamAttribute.max = doc[i][MAX].as<std::string>();
                 subParamAttribute.defaultValue = doc[i][DEFAULT].as<std::string>();
             }     
             subParamsAttiVector.push_back(subParamAttribute);
         }
         catch(YAML::Exception)
         {
             ROS_ERROR("LocalizationParameter get invalid tag");
         }
     }
     fin.close();
}