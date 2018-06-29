#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "fixpath_global_planner/fixpath_global_planner.h"

PLUGINLIB_DECLARE_CLASS(fixpath_global_planner, FixpathGlobalPlanner, fixpath_global_planner::FixpathGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace fixpath_global_planner{

FixpathGlobalPlanner::FixpathGlobalPlanner(): initialized_(false)
{
	ROS_ERROR("FixpathGlobalPlanner");
	ros::NodeHandle fixpath_planner_nh("fixpath_controller");
	fixpath_generate_client = fixpath_planner_nh.serviceClient<map_server::GetFixPath>("get_fixpath_point");
    tf_listener = new tf::TransformListener();
	initialize();
    ROS_ERROR("initialized finished!");
}

FixpathGlobalPlanner::~FixpathGlobalPlanner()
{
}

void FixpathGlobalPlanner::initialize()
{
	if(!initialized_){
		ROS_DEBUG("Init fixpath global planner");
		initialized_ = true;
	}
	else
		ROS_WARN("Fixpath global planner has already been initialized, you can't call it twice, doing nothing");
}

bool FixpathGlobalPlanner::adjustPlan(std::vector<geometry_msgs::PoseStamped>& plan,std::vector<geometry_msgs::PoseStamped>& new_plan)
{
    //get robot current pose
  geometry_msgs::PoseStamped msg_pose;
  tf::Stamped<tf::Pose> tmp_pose;
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                               tf::Vector3(0,0,0)), ros::Time(0), "base_link");
  try
  {
      tf_listener -> transformPose("map", ident, tmp_pose);
  }
  catch(tf::TransformException e)
  {
      ROS_DEBUG("Failed to compute current pose, skipping scan (%s)", e.what());
      return false;
  }
  tf::poseStampedTFToMsg(tmp_pose, msg_pose);

  double dx = 0.0;
  double dy = 0.0;
  double dist = 0.0;
  double min_dist = 10e8;
  int modify_index = 0;

  for(int i=0;i<plan.size();i++)
  {
      plan[i].header.frame_id = "map";
      plan[i].header.stamp = ros::Time::now();
      //ROS_ERROR("plan %.3f,%.3f,%.3f",plan[i].pose.position.x,plan[i].pose.position.y,tf::getYaw(plan[i].pose.orientation));
      dx = plan[i].pose.position.x - msg_pose.pose.position.x;
      dy = plan[i].pose.position.y - msg_pose.pose.position.y;
      dist = sqrt(dx * dx + dy * dy);
      //dth = tf::getYaw(plan[i].pose.orientation)) - tf::getYaw(msg_pose.pose.orientation));
      if( dist < min_dist)
      {
          min_dist = dist;
          modify_index = i;
      }
  }

  if(min_dist < 1.5)
  {
      //add pose to min dist pose
      geometry_msgs::PoseStamped p1 = msg_pose;
      double dt = 1/(min_dist/0.05);
      double theta = atan2(plan[modify_index].pose.position.y - msg_pose.pose.position.y,plan[modify_index].pose.position.x - msg_pose.pose.position.x);
      for(double t=0; t < 1;t+=dt)
      {
          p1.pose.position.x = msg_pose.pose.position.x * (1-t) + plan[modify_index].pose.position.x * t;
          p1.pose.position.y = msg_pose.pose.position.y * (1-t) + plan[modify_index].pose.position.y * t;
          p1.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
          new_plan.push_back(p1);
      }
      for(int i = modify_index;i<plan.size();i++)
      {
          new_plan.push_back(plan[i]);
      }
  }
  else
  {
      return false;
  }

  return true;
}

bool FixpathGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
	//request
	unsigned int path_id;
	unsigned int path_type;
    //response
	unsigned char path_found;
    bool flag = false;
	std::string error_message;
    std::vector<geometry_msgs::PoseStamped> path_tmp;

    path_id = (unsigned int)(start.pose.position.x);

	//get path_data by service
	map_server::GetFixPath srv;
	ROS_ERROR("FixpathGlobalPlanner::makePlan,id:%d",path_id);
    srv.request.path_id = path_id;
	srv.request.path_type = path_type;
    srv.request.path_scene_name = start.header.frame_id;
    srv.request.path_map_name = goal.header.frame_id;
	if(fixpath_generate_client.call(srv))
	{
		ROS_ERROR("fixpath get path_data operation ok!");
		path_found = srv.response.path_found;
		error_message = srv.response.error_message;
        path_tmp = srv.response.path;
        if(0 == path_found)
        {
            //modify fixpath because robot current pose
            flag = adjustPlan(path_tmp,plan);

            if(flag)
            {
                for(int i=0;i<plan.size();i++)
                {
                    plan[i].header.frame_id = "map";
                    plan[i].header.stamp = ros::Time::now();
                    ROS_ERROR("plan %.3f,%.3f,%.3f",plan[i].pose.position.x,plan[i].pose.position.y,tf::getYaw(plan[i].pose.orientation));
                }
                return true;
            }
            else
            {
		plan.push_back(start);
                return false;
            }
        }
        else
        {
            ROS_ERROR("fixpath did not found path failed!");
            return false;
        }
	}
	else
	{
		ROS_ERROR("fixpath get path_data operation failed!");
		return false;
	}
}
};
