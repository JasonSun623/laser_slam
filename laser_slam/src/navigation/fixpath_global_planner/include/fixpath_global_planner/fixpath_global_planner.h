#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <pcl_ros/publisher.h>
#include "map_server/GetFixPath.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/String.h"

namespace fixpath_global_planner{
	class FixpathGlobalPlanner : public nav_core::BaseGlobalPlanner{
		public:
			FixpathGlobalPlanner();
			void initialize();
			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){}
			bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
			bool getPlan(unsigned int path_id,std::vector<geometry_msgs::PoseStamped>& plan);
            bool adjustPlan(std::vector<geometry_msgs::PoseStamped>& plan,std::vector<geometry_msgs::PoseStamped>& new_plan);
			//void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);
			~FixpathGlobalPlanner();

		protected:
			boost::shared_ptr<FixpathGlobalPlanner> planner_;
			ros::Publisher plan_pub_;
			ros::ServiceClient fixpath_generate_client;
            //tf listener
            tf::TransformListener* tf_listener;
			bool initialized_;
	};
};
