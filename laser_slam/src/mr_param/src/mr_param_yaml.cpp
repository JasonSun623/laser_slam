#include "mr_param/mr_param_node.h"

static const char* CONFIG_PATH = "/home/robot/catkin_ws/src/mr_param/configs/";

MrParamYaml::MrParamYaml():priv_nh("~")
{
    priv_nh.param("config_path", config_path,std::string(CONFIG_PATH));
}

MrParamYaml::~MrParamYaml()
{
}

std::string MrParamYaml::getPkgsList()
{
    std::string path = config_path + "pkgs.yaml";
    return getData(path,"pkgs");
}

std::string MrParamYaml::getPkgParams(std::string pkgName)
{
    std::string path_attribute = config_path + pkgName +"_attribute.yaml";
    std::string path = config_path + pkgName +".yaml";
    return getData(path,getAttributeVector(path_attribute));
}

bool MrParamYaml::setPkgParams(std::string pkgName,std::string params)
{
    std::string path = config_path + pkgName +".yaml";
    std::string path_attribute = config_path + pkgName +"_attribute.yaml";
    std::vector<std::string> paramVector;
    boost::split(paramVector, params, boost::is_any_of(","), boost::token_compress_on);
    std::map<std::string,std::string> paramTypeMap;
    paramTypeMap = getParamType(path_attribute);
    return setParams(path,paramVector,paramTypeMap);
}

bool MrParamYaml::restorePkgParams(std::string pkgName)
{
    std::string path = config_path + pkgName +".yaml";
    std::string path_attribute = config_path + pkgName +"_attribute.yaml";
    std::vector<std::string> paramVector;
    paramVector = getParamDefault (path_attribute);
    std::map<std::string,std::string> paramTypeMap;
    paramTypeMap = getParamType(path_attribute);
    return setParams(path,paramVector,paramTypeMap);
}

std::string MrParamYaml::getData(std::string path,std::string data_name)
{
    std::string rlt;
    std::ifstream fin(path.c_str());

    // open fail
    if(!fin)
    {
        ROS_ERROR("fail open param yaml");
        return rlt;
    }
    YAML::Node doc = YAML::Load(fin);

    try
    {
       rlt = doc[data_name].as<std::string>();
    }
    catch(YAML::Exception)
    {
       ROS_ERROR("MrParam get invalid tag");
    }
    fin.close();
    return rlt;
}

std::string MrParamYaml::getData(std::string path,std::vector<ParamAttribute> data_attribute)
{
    std::string rlt;
    std::ifstream fin(path.c_str());
    std::string tempStr;
    // open fail
    if(!fin)
    {
        ROS_ERROR("fail open param yaml");
        return rlt;
    }
    YAML::Node doc = YAML::Load(fin);

    try
    {
        for(std::vector<ParamAttribute>::iterator it = data_attribute.begin(); it != data_attribute.end(); ++ it)
        {
            tempStr += (*it).name + ":" + doc[(*it).name].as<std::string>() + ",";
        }
        if(!tempStr.empty())
        {
            rlt = tempStr.substr(0,tempStr.size()-1);
        }
    }
    catch(YAML::Exception)
    {
       ROS_ERROR("MrParam get invalid tag");
    }
    fin.close();
    return rlt;
}

std::vector<ParamAttribute> MrParamYaml::getAttributeVector(std::string path)
{
    std::vector<ParamAttribute> paramAttributeVector;
    std::ifstream fin(path.c_str());
    // open fail
    if(!fin)
    {
        ROS_ERROR("fail open param attribute yaml");
        return paramAttributeVector;
    }
    YAML::Node doc = YAML::Load(fin);

    ParamAttribute paramAttribute;
    int itmeSize = doc.size();
    for(int i=0; i<itmeSize; i++)
    {
        try
        {
            paramAttribute.name = doc[i]["name"].as<std::string>();
            paramAttribute.type = doc[i]["type"].as<std::string>();
            if(0 == paramAttribute.type.compare("int")){
                 paramAttribute.defaultValue = doc[i]["default"].as<int>();
             }
             else if (0 == paramAttribute.type.compare("double"))
             {
                 paramAttribute.defaultValue = doc[i]["default"].as<double>();
             }
             else if (0 == paramAttribute.type.compare("bool"))
             {
                 paramAttribute.defaultValue = doc[i]["default"].as<bool>();
             }
             else if (0 == paramAttribute.type.compare("string"))
             {
                 paramAttribute.defaultValue = doc[i]["default"].as<std::string>();
             }     
             paramAttributeVector.push_back(paramAttribute);
         }
         catch(YAML::Exception)
         {
             ROS_ERROR("mr param attribute get invalid tag");
         }
     }
     fin.close();
     return paramAttributeVector;
}

std::map<std::string,std::string> MrParamYaml::getParamType(std::string path)
{
    std::map<std::string,std::string> paramTypeMap;
    std::ifstream fin(path.c_str());
    // open fail
    if(!fin)
    {
        ROS_ERROR("fail open param attribute yaml");
        return paramTypeMap;
    }
    YAML::Node doc = YAML::Load(fin);

    int itmeSize = doc.size();
    for(int i=0; i<itmeSize; i++)
    {
        try
        {
            paramTypeMap.insert(std::map<std::string,std::string>::value_type(
                doc[i]["name"].as<std::string>(), doc[i]["type"].as<std::string>()));
        }
        catch(YAML::Exception)
        {
            ROS_ERROR("LocalizationParameter get invalid tag");
        }
     }
     fin.close();
     return paramTypeMap;
}

bool MrParamYaml::setParams(std::string path,
                            std::vector<std::string> params,
                            std::map<std::string,std::string> param_type)
{
    std::string value;
    std::string name;
    std::string type;
    std::map<std::string,std::string>::iterator ptr;
    int tmp_int;
    double tmp_double;
    bool tmp_bool;

    std::ifstream fin(path.c_str());
    
    if(!fin)
    {
        ROS_ERROR("fail open param yaml");
        return false;
    }
    YAML::Node doc = YAML::Load(fin);
    fin.close();

    std::ofstream fout(path.c_str());
    if(!fout)
    {
        ROS_ERROR("fail open param yaml");
        return false;
    }
    for(std::vector<std::string>::iterator it = params.begin(); it != params.end(); ++ it)
    {
      
        name = (*it).substr(0,(*it).find(':'));
        value = (*it).substr((*it).find(':')+1);
        ptr = param_type.find(name);
        if(ptr != param_type.end()) 
        {
            type = (*ptr).second;
            if(0 == type.compare("int"))
            {
                std::istringstream(value) >> tmp_int;
                doc[name] = tmp_int;
            }
            else if (0 == type.compare("double"))
            {
                std::istringstream(value) >> tmp_double;
                doc[name] = tmp_double; 
            }
            else if (0 == type.compare("bool"))
            {
                std::istringstream(value) >> std::boolalpha >> tmp_bool;
                doc[name] = tmp_bool;
            }
            else if (0 == type.compare("string"))
            {
                doc[name] = value;
            }
        }       
    }
    try
    {       
        fout << doc; 
    }
    catch(YAML::Exception)
    {
       ROS_ERROR("mr param set invalid tag");
       fout.close();
       return false;
    }
    fout.close();
    return true;
}

std::vector<std::string> MrParamYaml::getParamDefault(std::string path)
{
    std::vector<std::string> paramsVector;
    std::ifstream fin(path.c_str());
    std::string tmpStr;
    // open fail
    if(!fin)
    {
        ROS_ERROR("fail open param attribute yaml");
        return paramsVector;
    }
    YAML::Node doc = YAML::Load(fin);

    int itmeSize = doc.size();
    for(int i=0; i<itmeSize; i++)
    {
        try
        {
            tmpStr = doc[i]["name"].as<std::string>()+": "+doc[i]["default"].as<std::string>();
            paramsVector.push_back(tmpStr);
        }
        catch(YAML::Exception)
        {
            ROS_ERROR("mr param default get invalid tag");
        }
     }
     fin.close();
     return paramsVector;
}