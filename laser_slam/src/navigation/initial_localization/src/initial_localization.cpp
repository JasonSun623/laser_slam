#include "initial_localization/initial_localization.h"


template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

InitialLocalization::InitialLocalization():initial_state_(READY)
{
    ros::NodeHandle n;
    current_pose_sub_ = n.subscribe("current_pose",20, &InitialLocalization::current_pose_callback,this);
    map_list_sub_ = n.subscribe("map_list_pub",10,&InitialLocalization::map_list_callback,this);
    app_pub_ = n.advertise<std_msgs::String>("app_pub",10);
    initial_pose_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    n.param("config_file_path",config_file_path_,std::string("/home/robot/catkin_ws/install/share/initial_localization/configs/config.yaml"));
}

bool InitialLocalization::read_config_file()
{
    std::ifstream fin(config_file_path_.c_str());
    if(!fin)
    {
	
        return false;
    }

    //load yaml file.
    config_info = YAML::Load(fin);
    if(config_info)
    {
        if(config_info["scene_name"] && config_info["pose_x"] && config_info["map_name"] &&
                config_info["pose_y"] && config_info["pose_th"])
        {
            double th = 0.0;
            config_info["pose_x"] >> latest_pose_.pose.pose.position.x;
            config_info["pose_y"] >> latest_pose_.pose.pose.position.y;
            config_info["pose_th"] >> th;
            tf::Quaternion q = tf::createQuaternionFromYaw(th);
            geometry_msgs::Quaternion qmsg;
            tf::quaternionTFToMsg(q, qmsg);
            latest_pose_.pose.pose.orientation = qmsg;
            config_info["scene_name"] >> scene_name_;
            config_info["map_name"] >> map_name_;
            return true;
        }
    }
    return false;
}

bool InitialLocalization::update_config_file()
{
    std::ofstream fout(config_file_path_.c_str());

    config_info["scene_name"] = scene_name_;
    config_info["map_name"] = map_name_;
    config_info["pose_x"] = latest_pose_.pose.pose.position.x;
    config_info["pose_y"] = latest_pose_.pose.pose.position.y;
    config_info["pose_th"] = tf::getYaw(latest_pose_.pose.pose.orientation);
    fout <<config_info;
    fout.close();

    return true;
}

void InitialLocalization::current_pose_callback(const nav_msgs::Odometry msg)
{
    if(initial_state_ != FINISHED)
    {
        ROS_ERROR("state is wrong");
        return;
    }
    double dist = hypot((msg.pose.pose.position.x - latest_pose_.pose.pose.position.x),
                        (msg.pose.pose.position.y - latest_pose_.pose.pose.position.y));
    double dth = tf::getYaw(latest_pose_.pose.pose.orientation) - tf::getYaw(msg.pose.pose.orientation);
    //ROS_INFO("dist:%f,dth:%f",dist,dth);
    if(dth > M_PI)
    {
        dth -= 2*M_PI;
    }
    else if(dth < -M_PI)
    {
        dth += 2*M_PI;
    }
    if((dist > 0.2) || (fabs(dth) > 0.2))
    {
        //write to file
        //ROS_INFO("write to file latest pose dist:%f,dth:%f",dist,dth);
        latest_pose_ = msg;
        update_config_file();
    }
    return;
}

setup_state_e InitialLocalization::get_setup_state()
{
    return initial_state_;
}

bool InitialLocalization::set_state(setup_state_e state)
{
    initial_state_ = state;
    return true;
}

bool InitialLocalization::load_map_pub()
{
    Json::Value value;
    value["pub_name"] = "map_load";
    value["publisher"] = "InitialLocalization";
    Json::Value data;
    data["scene_name"] = scene_name_;
    data["map_name"] = map_name_;
    value["data"] = data;

    std_msgs::String msg;
    msg.data = json_writer_.write(value);
    app_pub_.publish(msg);
    return true;
}

bool InitialLocalization::initial_pose_pub()
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = latest_pose_.pose.pose.position.x;
    pose.pose.pose.position.y = latest_pose_.pose.pose.position.y;
    pose.pose.pose.orientation = latest_pose_.pose.pose.orientation;

    pose.pose.covariance[6*0+0] = 0.5*0.5;
    pose.pose.covariance[6*1+1] = 0.5*0.5;
    pose.pose.covariance[6*3+3] = M_PI/12.0 * M_PI/12.0;
    initial_pose_pub_.publish(pose);
    ROS_INFO("publish initial pose");
    return true;
}
/*
 *
*/
void InitialLocalization::map_list_callback(const std_msgs::String msg)
{
    Json::Reader reader;
    Json::Value value;
    ROS_INFO("map list pub callback:%s",msg.data.c_str());
    if(reader.parse(msg.data,value) && (initial_state_ == FINISHED))
    {
        if(value["current_map_name"].isNull() || value["map_list"].isNull())
        {
            ROS_ERROR("current map and maplist is null");
            return;
        }
        Json::Value maplist = value["map_list"];
        bool flag = false;
        if(map_name_ != value["current_map_name"].asString())
        {
            ROS_INFO("update map name:%s",value["current_map_name"].asString().c_str());
            flag = true;
            map_name_ = value["current_map_name"].asString();
        }
        if(scene_name_ != maplist["scene_name"].asString())
        {
            ROS_INFO("update scene name:%s",maplist["scene_name"].asString().c_str());
            flag = true;
            scene_name_ = maplist["scene_name"].asString();
        }
        if(flag)
        {
            //write to file
            update_config_file();
            ROS_INFO("write to file");
        }
    }
    return;
}

int main(int argc, char **argv)
{
    bool flag = false;
    int delay_count = 0;


    ros::init(argc, argv, "initial_localization");

    InitialLocalization initial_localization;


    ros::Rate r(0.5);
    while(ros::ok())
    {
        ros::V_string str1;
        switch (initial_localization.get_setup_state())
        {
        case READY:
            {
                bool success = initial_localization.read_config_file();
                if(!success)
                {
                    initial_localization.latest_pose_.pose.pose.position.x = 0.0;
                    initial_localization.latest_pose_.pose.pose.position.y = 0.0;
                    tf::Quaternion q = tf::createQuaternionFromYaw(0.0);
                    geometry_msgs::Quaternion qmsg;
                    tf::quaternionTFToMsg(q, qmsg);
                    initial_localization.latest_pose_.pose.pose.orientation = qmsg;
                    initial_localization.map_name_ = "F001";
                    initial_localization.scene_name_ = "example";
                }
                initial_localization.update_config_file();
                initial_localization.set_state(WAIT_GET_CONFIG_FILE);
            }
            break;
        case WAIT_GET_CONFIG_FILE:
            flag = ros::master::getNodes(str1);
            if(flag)
            {
                int wait_count = 0;
                //ROS_INFO("nodes size:%d",str1.size());
                for(ros::V_string::iterator it = str1.begin();it != str1.end();it++)
                {

                    //ROS_INFO("nodes:%s",it->data());
                    if(0 == strcmp("/map_server_mrobot",it->data()))
                    {
                        ROS_INFO("map count++");
                        wait_count++;
                    }
                    else if(0 == strcmp("/amcl",it->data()))
                    {
                        ROS_INFO("amcl count++");
                        wait_count++;
                    }
                }
                ROS_INFO("wait count:%d",wait_count);
                if(2 == wait_count)
                {
                    //ROS_INFO("have launch map server and amcl");
                    initial_localization.set_state(WAIT_LOAD_MAP_FINISHED);
                }
                //ROS_INFO("have run amcl,%s",str1.begin()->data());
            }
            break;
        case WAIT_LOAD_MAP_FINISHED:
            initial_localization.load_map_pub();
            initial_localization.set_state(WAIT_SETUP_POSE);
            break;
        case WAIT_SETUP_POSE:
            initial_localization.initial_pose_pub();
            initial_localization.set_state(FINISHED);
            break;
        case FINISHED:
            break;
        default:
            break;
        }
        r.sleep();
        ros::spinOnce();
    }
    return 0;

}
