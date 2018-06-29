/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christian Connette
 *********************************************************************/

#include <mrobot_local_planner/mrobot_local_planner_ros.h>

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>


// register this planner as a BaseGlobalPlanner plugin
// (see http://www.ros.org/wiki/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
PLUGINLIB_DECLARE_CLASS(mrobot_local_planner, MrobotPlannerROS, mrobot_local_planner::MrobotPlannerROS, nav_core::BaseLocalPlanner)


  namespace mrobot_local_planner{

    MrobotPlannerROS::MrobotPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}


    MrobotPlannerROS::MrobotPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
      : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
      // initialize planner
      initialize(name, tf, costmap_ros);
    }


    MrobotPlannerROS::~MrobotPlannerROS() {}


    void MrobotPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
      // check if the plugin is already initialized
      if(!initialized_)
      {
        // copy adress of costmap and Transform Listener (handed over from move_base)
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        tf_ = tf;
        global_frame_ = costmap_ros_->getGlobalFrameID();
        world_model_ = new base_local_planner::CostmapModel(*(costmap_ros_->getCostmap()));
        footprint_spec_ = costmap_ros_->getRobotFootprint();

        inscribed_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
		circumscribed_radius_ = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
        // create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle pn("~/" + name);

        // read parameters from parameter server
        // get tolerances for "Target reached"
        pn.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
        pn.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);

        pn.param("look_head_dist", look_head_dist_, 1.2);
        pn.param("max_vx", max_vx_, 0.5);
        pn.param("max_vy", max_vy_, 0.0);
        pn.param("max_vth", max_vth_, 0.6);
        pn.param("max_acc_x", max_acc_x_, 1.0);
        pn.param("max_acc_y", max_acc_y_, 0.0);
        pn.param("max_acc_th", max_acc_th_, 1.0);
        pn.param("max_obstacle_dist", max_obstacle_dist_, 0.6);
        pn.param("max_obstacle_time", max_obstacle_time_, 0.5);
        pn.param("obstacle_detect_center", obstacle_detect_center_, 0.75);
        pn.param("obstacle_detect_max_radius", obstacle_detect_max_radius_, 0.5);
        pn.param("curvature_dist", curvature_dist_, 0.4);
        pn.param("max_rotate_adjust_th", max_rotate_adjust_th_, 0.3);
        pn.param("min_vth", min_vth_, 0.05);
        pn.param("control_freq", control_freq_, 20.0);
        pn.param("min_stop_dist", min_stop_dist_, 0.05);
        actual_max_vx_ = max_vx_;
        actual_max_vy_ = max_vy_;
        actual_max_vth_ = max_vth_;
        update_flag_ = false;

		last_stop_time = ros::Time::now();
        // set lower bound for velocity -> if velocity in this region stop! (to avoid limit-cycles or lock)
        pn.param("rot_stopped_vel", rot_stopped_vel_, 1e-2);
        pn.param("trans_stopped_vel", trans_stopped_vel_, 1e-2);

        // advertise topics (adapted global plan and predicted local trajectory)
        g_plan_pub_ = pn.advertise<nav_msgs::Path>("global_plan", 1);
        l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);
        param_update_srv_ = pn.advertiseService("fixpath_params_update",&MrobotPlannerROS::updateParams,this);


        // subscribe to topics (to get odometry information, we need to get a handle to the topic in the global namespace)
        ros::NodeHandle gn;
        odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&MrobotPlannerROS::odomCallback, this, _1));


        // create the actual planner that we'll use. Pass Name of plugin and pointer to global costmap to it.
        // (configuration is done via parameter server)
        /*eband_ = boost::shared_ptr<MrobotPlanner>(new MrobotPlanner(name, costmap_ros_));

        // create the according controller
        eband_trj_ctrl_ = boost::shared_ptr<MrobotTrajectoryCtrl>(new MrobotTrajectoryCtrl(name, costmap_ros_));

        // create object for visualization
        eband_visual_ = boost::shared_ptr<MrobotVisualization>(new MrobotVisualization);

        // pass visualization object to elastic band
        eband_->setVisualization(eband_visual_);

        // pass visualization object to controller
        eband_trj_ctrl_->setVisualization(eband_visual_);

        // initialize visualization - set node handle and pointer to costmap
        eband_visual_->initialize(pn, costmap_ros);*/


        // set initialized flag
        initialized_ = true;
        xy_tolerance_latch_ = false;

        // this is only here to make this process visible in the rxlogger right from the start
        ROS_INFO("Mrobot local planner initialized.");
      }
      else
      {
        ROS_WARN("This planner has already been initialized, doing nothing.");
      }
    }

    bool MrobotPlannerROS::updateParams(motion_planner::MotionOp::Request &req, motion_planner::MotionOp::Response &res)
    {
        std::string test =req.cmd;
        Json::Reader reader;
        Json::Value value;
        Json::Value data;
        ROS_INFO("params:%s",test.c_str());
        if(reader.parse(test,value))
        {
           data = value["params"];
           if(data["max_vel_x"].isDouble())
           {
               update_max_vel_x_ = data["max_vel_x"].asDouble();
               ROS_INFO("update_max_vel_x value is %f",update_max_vel_x_);
           }
           if(data["max_vel_th"].isDouble())
           {
               update_max_vel_th_ = data["max_vel_th"].asDouble();
               ROS_INFO("update_max_vel_th value is %f",update_max_vel_th_);
           }
           if(data["max_acc_x"].isDouble())
           {
               update_acc_x_ = data["max_acc_x"].asDouble();
               ROS_INFO("update_acc_x value is %f",update_acc_x_);
           }
           if(data["max_acc_th"].isDouble())
           {
               update_acc_th_ = data["max_acc_th"].asDouble();
               ROS_INFO("max_update_acc_th_ value is %f",update_acc_th_);
           }
           if(data["collision_stop_dist"].isDouble())
           {
               update_min_stop_dist_ = data["collision_stop_dist"].asDouble();
               ROS_INFO("update_min_stop_dist value is %f",update_min_stop_dist_);
           }
           update_flag_ = true;
           return true;
       }
       return false;
    }

    // set global plan to wrapper and pass it to eband
    bool MrobotPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {

      // check if plugin initialized
      if(!initialized_)
      {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
      }

      //reset the global plan
      global_plan_.clear();
      global_plan_ = orig_global_plan;
      xy_tolerance_latch_ = false;

      // transform global plan to the map frame we are working in
      // this also cuts the plan off (reduces it to local window)
      /*std::vector<int> start_end_counts (2, (int) global_plan_.size()); // counts from the end() of the plan
      if(!mrobot_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan_, start_end_counts))
      {
        // if plan could not be tranformed abort control and local planning
        ROS_WARN("Could not transform the global plan to the frame of the controller");
        return false;
      }

      // also check if there really is a plan
      if(transformed_plan_.empty())
      {
        // if global plan passed in is empty... we won't do anything
        ROS_WARN("Transformed plan is empty. Aborting local planner!");
        return false;
      }

      // set plan - as this is fresh from the global planner robot pose should be identical to start frame
      if(!eband_->setPlan(transformed_plan_))
      {
        // We've had some difficulty where the global planner keeps returning a valid path that runs through an obstacle
        // in the local costmap. See issue #5. Here we clear the local costmap and try one more time.
        costmap_ros_->resetLayers();
        if (!eband_->setPlan(transformed_plan_)) {
          ROS_ERROR("Setting plan to Elastic Band method failed!");
          return false;
        }
      }
      ROS_INFO("Global plan set to elastic band for optimization");

      // plan transformed and set to elastic band successfully - set counters to global variable
      plan_start_end_counter_ = start_end_counts;

      // let eband refine the plan before starting continuous operation (to smooth sampling based plans)
      eband_->optimizeBand();


      // display result
      std::vector<mrobot_local_planner::Bubble> current_band;
      if(eband_->getBand(current_band))
        eband_visual_->publishBand("bubbles", current_band);/**/

      ROS_ERROR("set a new path");
      // set goal as not reached
      goal_reached_ = false;

      return true;
    }


    /*bool MrobotPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
      // check if plugin initialized
      if(!initialized_)
      {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
      }

      // instantiate local variables
      //std::vector<geometry_msgs::PoseStamped> local_plan;
      tf::Stamped<tf::Pose> global_pose;
      geometry_msgs::PoseStamped global_pose_msg;
      std::vector<geometry_msgs::PoseStamped> tmp_plan;

      // get curent robot position
      ROS_DEBUG("Reading current robot Position from costmap and appending it to elastic band.");
      if(!costmap_ros_->getRobotPose(global_pose))
      {
        ROS_WARN("Could not retrieve up to date robot pose from costmap for local planning.");
        return false;
      }

      // convert robot pose to frame in plan and set position in band at which to append
      tf::poseStampedTFToMsg(global_pose, global_pose_msg);
      tmp_plan.assign(1, global_pose_msg);
      mrobot_local_planner::AddAtPosition add_frames_at = add_front;

      // set it to elastic band and let eband connect it
      if(!eband_->addFrames(tmp_plan, add_frames_at))
      {
        ROS_WARN("Could not connect robot pose to existing elastic band.");
        return false;
      }

      // get additional path-frames which are now in moving window
      ROS_DEBUG("Checking for new path frames in moving window");
      std::vector<int> plan_start_end_counter = plan_start_end_counter_;
      std::vector<geometry_msgs::PoseStamped> append_transformed_plan;
      // transform global plan to the map frame we are working in - careful this also cuts the plan off (reduces it to local window)
      if(!mrobot_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan_, plan_start_end_counter))
      {
        // if plan could not be tranformed abort control and local planning
        ROS_WARN("Could not transform the global plan to the frame of the controller");
        return false;
      }

      // also check if there really is a plan
      if(transformed_plan_.empty())
      {
        // if global plan passed in is empty... we won't do anything
        ROS_WARN("Transformed plan is empty. Aborting local planner!");
        return false;
      }

      ROS_DEBUG("Retrieved start-end-counts are: (%d, %d)", plan_start_end_counter.at(0), plan_start_end_counter.at(1));
      ROS_DEBUG("Current start-end-counts are: (%d, %d)", plan_start_end_counter_.at(0), plan_start_end_counter_.at(1));

      // identify new frames - if there are any
      append_transformed_plan.clear();
      // did last transformed plan end futher away from end of complete plan than this transformed plan?
      if(plan_start_end_counter_.at(1) > plan_start_end_counter.at(1)) // counting from the back (as start might be pruned)
      {
        // new frames in moving window
        if(plan_start_end_counter_.at(1) > plan_start_end_counter.at(0)) // counting from the back (as start might be pruned)
        {
          // append everything
          append_transformed_plan = transformed_plan_;
        }
        else
        {
          // append only the new portion of the plan
          int discarded_frames = plan_start_end_counter.at(0) - plan_start_end_counter_.at(1);
          ROS_ASSERT(transformed_plan_.begin() + discarded_frames + 1 >= transformed_plan_.begin());
          ROS_ASSERT(transformed_plan_.begin() + discarded_frames + 1 < transformed_plan_.end());
          append_transformed_plan.assign(transformed_plan_.begin() + discarded_frames + 1, transformed_plan_.end());
        }

        // set it to elastic band and let eband connect it
        ROS_DEBUG("Adding %d new frames to current band", (int) append_transformed_plan.size());
        add_frames_at = add_back;
        if(eband_->addFrames(append_transformed_plan, add_back))
        {
          // appended frames succesfully to global plan - set new start-end counts
          ROS_DEBUG("Sucessfully added frames to band");
          plan_start_end_counter_ = plan_start_end_counter;
        }
        else {
          ROS_WARN("Failed to add frames to existing band");
          return false;
        }
      }
      else
        ROS_DEBUG("Nothing to add");

      // update Elastic Band (react on obstacle from costmap, ...)
      ROS_DEBUG("Calling optimization method for elastic band");
      std::vector<mrobot_local_planner::Bubble> current_band;
      if(!eband_->optimizeBand())
      {
        ROS_WARN("Optimization failed - Band invalid - No controls availlable");
        // display current band
        if(eband_->getBand(current_band))
          eband_visual_->publishBand("bubbles", current_band);
        return false;
      }

      // get current Elastic Band and
      eband_->getBand(current_band);
      // set it to the controller
      if(!eband_trj_ctrl_->setBand(current_band))
      {
        ROS_DEBUG("Failed to to set current band to Trajectory Controller");
        return false;
      }

      // set Odometry to controller
      if(!eband_trj_ctrl_->setOdometry(base_odom_))
      {
        ROS_DEBUG("Failed to to set current odometry to Trajectory Controller");
        return false;
      }

      // get resulting commands from the controller
      geometry_msgs::Twist cmd_twist;
      if(!eband_trj_ctrl_->getTwist(cmd_twist, goal_reached_))
      {
        ROS_DEBUG("Failed to calculate Twist from band in Trajectory Controller");
        return false;
      }


      // set retrieved commands to reference variable
      ROS_DEBUG("Retrieving velocity command: (%f, %f, %f)", cmd_twist.linear.x, cmd_twist.linear.y, cmd_twist.angular.z);
      cmd_vel = cmd_twist;


      // publish plan
      std::vector<geometry_msgs::PoseStamped> refined_plan;
      if(eband_->getPlan(refined_plan))
        // TODO publish local and current gloabl plan
        base_local_planner::publishPlan(refined_plan, g_plan_pub_);
      //base_local_planner::publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);

      // display current band
      if(eband_->getBand(current_band))
        eband_visual_->publishBand("bubbles", current_band);

      return true;
    }*/

    bool MrobotPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
       static double tmp_dth = 0.0;
      // check if plugin initialized
      if(!initialized_)
      {
          ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
          return false;
      }

      if(update_flag_)
      {
          max_vx_ = update_max_vel_x_;
          max_vth_ = update_max_vel_th_;
          max_acc_x_ = update_acc_x_;
          max_acc_th_ = update_acc_th_;
          min_stop_dist_ = update_min_stop_dist_;
          update_flag_ = false;
          ROS_INFO("calculate max vx,vth,dist:%f,%f,%f",max_vx_,max_vth_,min_stop_dist_);
      }
      nav_msgs::Odometry base_odom;
      {
      	boost::mutex::scoped_lock lock(odom_mutex_);
      	base_odom = base_odom_;
      }
      //ROS_ERROR("compute vel 1");
      // instantiate local variables
      //std::vector<geometry_msgs::PoseStamped> local_plan;
      tf::Stamped<tf::Pose> global_pose;

      // get curent robot position
      //ROS_DEBUG("Reading current robot Position from costmap and appending it to elastic band.");
      if(!costmap_ros_->getRobotPose(global_pose))
      {
          ROS_WARN("Could not retrieve up to date robot pose from costmap for local planning.");
          return false;
      }

      //ROS_ERROR("compute vel 2");
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      //get the global plan in our local planner frame.
      if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan))
      {
          ROS_WARN("Could not transform the global plan to the frame of the controller");
          return false;
      }
      //ROS_ERROR("compute vel 3");
      if(!modify_path(global_pose,transformed_plan,global_plan_))
      {
	  ROS_WARN("modify path return error ");
          return false;
      }
      //ROS_ERROR("compute vel 4");
      if(transformed_plan.empty())
      {
	  ROS_WARN("modify path made transformed_plan empty");
          return false;
      }
      //get global pose
      tf::Stamped<tf::Pose> robot_pose;
      tf_->transformPose(global_plan_.begin()->header.frame_id, global_pose, robot_pose);


      double dx = robot_pose.getOrigin().x() - global_plan_.back().pose.position.x;
      double dy = robot_pose.getOrigin().y() - global_plan_.back().pose.position.y;
      double dth = tf::getYaw(global_plan_.back().pose.orientation) - tf::getYaw(robot_pose.getRotation());
      dth = adjust_th(dth);

      ros::NodeHandle n("~");
      if (!n.getParam ("xy_goal_tolerance", xy_goal_tolerance_))
        xy_goal_tolerance_ = 0.1;
      if (!n.getParam ("yaw_goal_tolerance", yaw_goal_tolerance_))
        yaw_goal_tolerance_ = 0.05;

      ROS_INFO("calculate a new vel:%f,dth:%f",sqrt(dx*dx + dy*dy),dth);
      if (xy_tolerance_latch_ || (sqrt(dx*dx + dy*dy) <= xy_goal_tolerance_))
      {

          //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
          //just rotate in place
          xy_tolerance_latch_ = true;

          //check to see if the goal orientation has been reached
          if (fabs(dth) <= yaw_goal_tolerance_)
          {
              //set the velocity command to zero
              cmd_vel.linear.x = 0.0;
              cmd_vel.linear.y = 0.0;
              cmd_vel.angular.z = 0.0;
              //rotating_to_goal_ = false;
              xy_tolerance_latch_ = false;
              goal_reached_ = true;
              tmp_dth = 0.0;
          }
          else
          {
              double vx_scale = 1.0;
              double vy_scale = 1.0;
              double vth_scale = 1.0;
              calculateMaxVelScale(global_pose,vx_scale,vy_scale,vth_scale);
              tmp_dth += dth;
              if(fabs(tmp_dth) > M_PI*5)
              {
                  tmp_dth = tmp_dth/fabs(tmp_dth)*M_PI*5;
              }
              double vth = 0.0;
              if(fabs(dth) > max_rotate_adjust_th_)
              {
                  vth = 0.45*dth + tmp_dth*0.001;
              }
              else
              {
                  vth = 0.8*dth + tmp_dth*0.001;
              }
              vth = 0.65*dth + tmp_dth*0.001;
              if(vth > max_vth_*vth_scale)
              {
                  vth = max_vth_*vth_scale;
              }
              else if(vth < -max_vth_*vth_scale)
              {
                  vth = -max_vth_*vth_scale;
              }
              if(vth > base_odom.twist.twist.angular.z + max_acc_th_/control_freq_)
              {
                  vth = base_odom.twist.twist.angular.z + max_acc_th_/control_freq_;
              }
              else if(vth < base_odom.twist.twist.angular.z - max_acc_th_/control_freq_)
              {
                  vth = base_odom.twist.twist.angular.z - max_acc_th_/control_freq_;
              }
              cmd_vel.linear.x = 0;
              cmd_vel.linear.y = 0;
              cmd_vel.angular.z = vth;
          }
          // check cmd vel
          //ROS_INFO("check rotate vel,%f",cmd_vel.angular.z);
          return checkCollision(cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z,DISTANCE_METHOD);
      }


      //calculate robot vel based on transform plan and robot current pose.
      bool flag = calculateVel(cmd_vel,global_pose,transformed_plan, base_odom);

      ROS_INFO("final compute vel,x:%f,th:%f",cmd_vel.linear.x,cmd_vel.angular.z);
      return flag;
    }


    bool MrobotPlannerROS::isGoalReached()
    {
      // check if plugin initialized
      if(!initialized_)
      {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
      }

      return goal_reached_;

    }


    void MrobotPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      // lock Callback while reading data from topic
      boost::mutex::scoped_lock lock(odom_mutex_);

      // get odometry and write it to member variable (we assume that the odometry is published in the frame of the base)
      base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
      base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
      base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    }

    bool MrobotPlannerROS::calculateVel(geometry_msgs::Twist& cmd_vel,const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Odometry& base_odom)
    {

        static double last_dth = 0.0;
        static double acc_dth = 0.0;
        std::vector<geometry_msgs::PoseStamped>::iterator min_it=plan.begin();
        std::vector<geometry_msgs::PoseStamped>::iterator it=plan.begin();
        double dist = 10e8;
        for(it=plan.begin();it != plan.end();it++)
        {
            double dx = global_pose.getOrigin().getX() - it->pose.position.x;
            double dy = global_pose.getOrigin().getY() - it->pose.position.y;
            if(sqrt(dx*dx + dy*dy) < dist)
            {
                dist = sqrt(dx*dx + dy*dy);
                min_it = it;
            }
        }

        dist = base_odom.twist.twist.linear.x*base_odom.twist.twist.linear.x/(2*max_acc_x_);
        it=min_it;
        for(;it != plan.end();it++)
        {
            double dx = global_pose.getOrigin().getX() - it->pose.position.x;
            double dy = global_pose.getOrigin().getY() - it->pose.position.y;
            if(sqrt(dx*dx + dy*dy) > look_head_dist_+dist)
            {
                break;
            }
        }

        tf::Stamped<tf::Pose> target_pose;
        if(it == plan.end())
        {
            ROS_ERROR("could not find look head dist,us last one");
            it = plan.end()-1;
        }
        tf::poseStampedMsgToTF(*it,target_pose);



        double path_curvature = 0.0;
        calculateActualMaxVel(global_pose,plan,path_curvature);
        //ROS_INFO("actual max vx and vth:%f,%f",actual_max_vx_,actual_max_vth_);



        ROS_DEBUG("current pose,x,y,th:%.3f,%.3f,%.3f",global_pose.getOrigin().getX(),global_pose.getOrigin().getY(),tf::getYaw(global_pose.getRotation()));
        ROS_DEBUG("target_pose pose,x,y,th:%.3f,%.3f,%.3f",target_pose.getOrigin().getX(),target_pose.getOrigin().getY(),tf::getYaw(target_pose.getRotation()));
        ROS_DEBUG("min pose,x,y,th:%.3f,%.3f,%.3f",min_it->pose.position.x,min_it->pose.position.y,tf::getYaw(min_it->pose.orientation));
        dist = (target_pose.getOrigin() - global_pose.getOrigin()).length();
        double path_th = atan2(target_pose.getOrigin().getY() - global_pose.getOrigin().getY(),target_pose.getOrigin().getX() - global_pose.getOrigin().getX());
        double dth = adjust_th(path_th - tf::getYaw(global_pose.getRotation()));
        double path_2_th = atan2(min_it->pose.position.y-global_pose.getOrigin().getY(),min_it->pose.position.x-global_pose.getOrigin().getX());
        path_2_th = -adjust_th(path_th - path_2_th);
        ROS_DEBUG("cal vel dth:%f",dth);
        if(fabs(dth) > max_rotate_adjust_th_)
        {

            double vth = 0.0;
            acc_dth += dth;
            if(fabs(acc_dth) > M_PI*5)
            {
                acc_dth = acc_dth/fabs(acc_dth)*M_PI*5;
            }
            if(fabs(dth) > max_rotate_adjust_th_*5.0)
            {
                vth = 0.45*dth+acc_dth*0.001;
            }
            else if(fabs(dth) > max_rotate_adjust_th_*2.0)
            {
                vth = 0.5*dth+acc_dth*0.001;
            }
            else
            {
                vth = 0.5*dth+acc_dth*0.001;
            }
            if(vth > actual_max_vth_)
            {
                vth = actual_max_vth_;
            }
            else if(vth < -actual_max_vth_)
            {
                vth = -actual_max_vth_;
            }
            if(fabs(vth) < min_vth_)
            {
                vth = vth/fabs(vth)*min_vth_;
            }

            if(vth > base_odom.twist.twist.angular.z + max_acc_th_/control_freq_)
            {
                vth = base_odom.twist.twist.angular.z + max_acc_th_/control_freq_;
            }
            else if(vth < base_odom.twist.twist.angular.z - max_acc_th_/control_freq_)
            {
                vth = base_odom.twist.twist.angular.z - max_acc_th_/control_freq_;
            }

            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = vth;
            dth_acc_ = 0.0;
        }
        else
        {
            acc_dth = 0.0;
            double to_goal_dist = 0.0;
            //calculate path distance to goal.
            {
                double dx = global_pose.getOrigin().getX() - plan.back().pose.position.x;
                double dy = global_pose.getOrigin().getY() - plan.back().pose.position.y;
                to_goal_dist = sqrt(dx*dx + dy*dy);
                ROS_INFO("to goal dist:%f",to_goal_dist);
                if(to_goal_dist < 0.5)
                {
                    if(actual_max_vx_ >= 0.15)
                    {
                        actual_max_vx_ = 0.15;
                    }
                }
                else if(to_goal_dist < 1.0)
                {
                    if(actual_max_vx_ >= 0.3)
                    {
                        actual_max_vx_ = 0.3;
                    }
                }
            }
            double curvature_scale = 1/(fabs(path_curvature) + 1);
            double vx = curvature_scale*actual_max_vx_*(cos(fabs(dth)*M_PI/max_rotate_adjust_th_)+1)*0.5;
            if(fabs(vx) < 0.02)
            {
                vx = 0;
            }
            dth_acc_ += dth;
            double dth_vth = dth*0.85 + (dth - last_dth)*0.0 + dth_acc_*0.005;
            if(fabs(dth_vth) > 0.1)
            {
                dth_vth = dth_vth/fabs(dth_vth)*0.1;
            }
            double vth = vx*path_curvature + dth_vth;
            //double vth = vx*path_curvature + actual_max_vth_*(sin(dth*M_PI_2*3.3)*0.9 + path_2_th/M_PI*0.1)*0.1;
            if(fabs(vth) > actual_max_vth_)
            {
                vx = actual_max_vth_/fabs(vth)*vx;
            }
            //double vth = actual_max_vth_*(sin(dth*M_PI_2*3.3)*0.5 + path_2_th/M_PI*0.5);
            //ROS_INFO("calcualte cmd vx:%f,base actual vx:%f",vx,base_odom.twist.twist.linear.x);
            if(vx > base_odom.twist.twist.linear.x + max_acc_x_/control_freq_)
            {
                vx = base_odom.twist.twist.linear.x + max_acc_x_/control_freq_;
            }
            else if(vx < base_odom.twist.twist.linear.x - max_acc_x_/control_freq_)
            {
                vx = base_odom.twist.twist.linear.x - max_acc_x_/control_freq_;
            }

            if(vth > base_odom.twist.twist.angular.z + max_acc_th_/control_freq_)
            {
                vth = base_odom.twist.twist.angular.z + max_acc_th_/control_freq_;
            }
            else if(vth < base_odom.twist.twist.angular.z - max_acc_th_/control_freq_)
            {
                vth = base_odom.twist.twist.angular.z - max_acc_th_/control_freq_;
            }
            cmd_vel.linear.x = vx;
            cmd_vel.angular.z = vth;
        }
        last_dth = dth;
        ROS_INFO("calculate vel,vx,vth:%f,%f",cmd_vel.linear.x,cmd_vel.angular.z);
        return checkCollision(cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z,DISTANCE_METHOD);
    }
    bool MrobotPlannerROS::modify_path(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan,std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
	  if(plan.empty())
          {
	      ROS_WARN("modify path received plan empty");
              return false;
          }
          ROS_ASSERT(global_plan.size() >= plan.size());
          std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
          std::vector<geometry_msgs::PoseStamped>::iterator global_it = global_plan.begin();
          double min_dist = 10e8;
          std::vector<geometry_msgs::PoseStamped>::iterator min_it;
          int i = 0;
          int min_index = 0;

          while(it != plan.end())
          {
              const geometry_msgs::PoseStamped& w = *it;
              // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
              double x_diff = global_pose.getOrigin().x() - w.pose.position.x;
              double y_diff = global_pose.getOrigin().y() - w.pose.position.y;
              double distance_sq = x_diff * x_diff + y_diff * y_diff;
              if(distance_sq < min_dist)
              {
                  ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.getOrigin().x(), global_pose.getOrigin().y(), w.pose.position.x, w.pose.position.y);
                  min_dist = distance_sq;
                  min_index = i;
              }
              it++;
              i++;
          }
          if(0 != min_index)
          {
              plan.erase(plan.begin(),plan.begin()+min_index-1);
              global_plan.erase(global_plan.begin(),global_plan.begin()+min_index-1);
          }
          //ROS_INFO("erase min index point:%d",min_index);
          return true;
    }

    bool MrobotPlannerROS::transformGlobalPlan(const tf::TransformListener& tf,const std::vector<geometry_msgs::PoseStamped>& global_plan,
        const tf::Stamped<tf::Pose>& global_pose,const costmap_2d::Costmap2D& costmap,const std::string& global_frame,
        std::vector<geometry_msgs::PoseStamped>& transformed_plan)
    {
      transformed_plan.clear();

      if (global_plan.empty()) {
        ROS_ERROR("Received plan with zero length");
        return false;
      }

      const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
      try {
        // get plan_to_global_transform from plan frame to global_frame
        tf::StampedTransform plan_to_global_transform;
        tf.waitForTransform(global_frame, ros::Time::now(),
                            plan_pose.header.frame_id, plan_pose.header.stamp,
                            plan_pose.header.frame_id, ros::Duration(0.5));
        tf.lookupTransform(global_frame, ros::Time(),
                           plan_pose.header.frame_id, plan_pose.header.stamp,
                           plan_pose.header.frame_id, plan_to_global_transform);

        //let's get the pose of the robot in the frame of the plan
        tf::Stamped<tf::Pose> robot_pose;
        tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);

        //we'll discard points on the plan that are outside the local costmap
        double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                         costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

        unsigned int i = 0;
        double sq_dist_threshold = dist_threshold * dist_threshold;
        double sq_dist = 0;

        //we need to loop to a point on the plan that is within a certain distance of the robot
        while(i < (unsigned int)global_plan.size()) {
          double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
          double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
          sq_dist = x_diff * x_diff + y_diff * y_diff;
          if (sq_dist <= sq_dist_threshold) {
            break;
          }
          ++i;
        }

        tf::Stamped<tf::Pose> tf_pose;
        geometry_msgs::PoseStamped newer_pose;

        //now we'll transform until points are outside of our distance threshold
        while(i < (unsigned int)global_plan.size() && sq_dist <= sq_dist_threshold) {
          const geometry_msgs::PoseStamped& pose = global_plan[i];
          poseStampedMsgToTF(pose, tf_pose);
          tf_pose.setData(plan_to_global_transform * tf_pose);
          tf_pose.stamp_ = plan_to_global_transform.stamp_;
          tf_pose.frame_id_ = global_frame;
          poseStampedTFToMsg(tf_pose, newer_pose);

          transformed_plan.push_back(newer_pose);

          double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
          double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
          sq_dist = x_diff * x_diff + y_diff * y_diff;

          ++i;
        }
      }
      catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
      }
      catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
      }
      catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        if (!global_plan.empty())
          ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

        return false;
      }

      return true;
    }

    double MrobotPlannerROS::adjust_th(double th)
    {
        while(th > M_PI)
        {
            th -= 2*M_PI;
        }
        while(th <= -M_PI)
        {
            th += 2*M_PI;
        }
        return th;
    }
    void MrobotPlannerROS::stopWithAcc(double &vx, double &vy, double &vth)
    {
        if(vx > max_acc_x_/control_freq_)
        {
            vx = vx - max_acc_x_/control_freq_;
        }
        else if(vx < - max_acc_x_/control_freq_)
        {
            vx = vx + max_acc_x_/control_freq_;
        }
        //else
        //{
        //    vx = 0.0;
        //}
        if(vx > base_odom_.twist.twist.linear.x + max_acc_x_/control_freq_)
        {
             vx = base_odom_.twist.twist.linear.x + max_acc_x_/control_freq_;
        }
        else if(vx < base_odom_.twist.twist.linear.x - max_acc_x_/control_freq_)
        {
             vx = base_odom_.twist.twist.linear.x - max_acc_x_/control_freq_;
        }

        if(vth >  max_acc_th_/control_freq_)
        {
            vth = vth - max_acc_th_/control_freq_;
        }
        else if(vth < - max_acc_th_/control_freq_)
        {
            vth = vth + max_acc_th_/control_freq_;
        }
        //else
        //{
        //    vth = 0.0;
        //}
        if(vth > base_odom_.twist.twist.angular.z + max_acc_th_/control_freq_)
        {
             vth = base_odom_.twist.twist.angular.z + max_acc_th_/control_freq_;
        }
        else if(vth < base_odom_.twist.twist.angular.z - max_acc_th_/control_freq_)
        {
             vth = base_odom_.twist.twist.angular.z - max_acc_th_/control_freq_;
        }
    }
    bool MrobotPlannerROS::checkCollision(double& vx, double& vy, double& vth,collision_method_e method)
    {
        tf::Stamped<tf::Pose> global_pose;
        if (!costmap_ros_->getRobotPose(global_pose))
        {
            return false;
        }

        tf::StampedTransform base_to_odom;
        ros::Time t = ros::Time::now();
        try
        {
            tf_listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(0.2));
            tf_listener.lookupTransform ("base_link", "odom", ros::Time(0), base_to_odom);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Could not get initial transform from base to odom frame, %s", ex.what());
            return false;
        }
        if(DISTANCE_METHOD == method)
        {
            double obs_dist = min_stop_dist_;
			//if(base_odom_.twist.twist.linear.x > 0.1)
			//obs_dist+= fabs(base_odom_.twist.twist.linear.x/max_vx_)*max_obstacle_dist_;
			obs_dist+= fabs(base_odom_.twist.twist.linear.x*max_vx_*max_obstacle_dist_);
			ROS_INFO("odom_vx:%f, odom_vth:%f, obs_dist:%f, max_vx_:%f, max_obstacle_dist_:%f", base_odom_.twist.twist.linear.x, base_odom_.twist.twist.angular.z, obs_dist, max_vx_, max_obstacle_dist_);
            if(fabs(vth) > 0.001 && fabs(vx) > 0.001)
            {
                double r = vx/vth;
                double s = 0.0;
                double ds = 0.04;
                while(s <= (obs_dist + 0.1))
                {
                    double theta = s/r;
                    double x = sin(theta)*r;
                    double y = r * (1-cos(theta));

                    tf::Transform d_tf , d_tf_new;
                    createTfFromXYTheta(x, y, 0, d_tf);
                    d_tf_new = base_to_odom.inverse() * d_tf * base_to_odom;

                    double tmp_x = global_pose.getOrigin().getX() + d_tf_new.getOrigin().x();
                    double tmp_y = global_pose.getOrigin().getY() + d_tf_new.getOrigin().y();
                    double tmp_th = tf::getYaw(global_pose.getRotation()) + theta;
                    if (tmp_th >= M_PI)
                    {
                        tmp_th -= 2.0 * M_PI;
                    }
                    else if (tmp_th < -M_PI)
                    {
                        tmp_th += 2.0 * M_PI;
                    }

                    double footprint_cost = world_model_->footprintCost(tmp_x, tmp_y, tmp_th,footprint_spec_,inscribed_radius_,circumscribed_radius_);
                    if(footprint_cost < 0)
                    {
                        ROS_INFO("stop move when vx,%f,vth:%f,dist:%f in whole:%f",vx,vth,s,obs_dist);
                        //emergency stop with acc
	                    double actual_stop_dist = min_stop_dist_;
                        ROS_INFO("last stop:%f,now:%f",last_stop_time.toSec(),ros::Time::now().toSec());
						if((last_stop_time + ros::Duration(5.0) > ros::Time::now()) && (min_stop_dist_ > 0.1))
						{
						  actual_stop_dist = min_stop_dist_ + 0.1;
						}
						ROS_INFO("actual_stop_dist:%f", actual_stop_dist);
						if(s > actual_stop_dist)
						{
							stopWithAcc(vx,vy,vth);
							return true;
						}
						else
						{
						    last_stop_time = ros::Time::now();
							vx = 0.0;
							vy = 0.0;
							vth = 0.0;
                            ROS_INFO("last stop time is now");
						}
                        return false;
                    }
                    s += ds;
                }
            }
            else if(fabs(vth) > 0.001)
            {
                double th = 0.0;
                double dth = 0.04;
                while(fabs(th) < 0.35)
                {

                    double tmp_x = global_pose.getOrigin().getX();
                    double tmp_y = global_pose.getOrigin().getY();
                    double tmp_th = tf::getYaw(global_pose.getRotation()) + th;
                    double footprint_cost = world_model_->footprintCost(tmp_x, tmp_y, tmp_th,footprint_spec_,inscribed_radius_,circumscribed_radius_);
                    if(footprint_cost < 0)
                    {
                        ROS_INFO("stop move rotate when vx,%f,vth:%f,th:%f",vx,vth,th);
                        //emergency stop with acc
                        //stopWithAcc(vx,vy,vth);
                        vx = 0.0;
                        vy = 0.0;
                        vth = 0.0;
                        return false;
                    }
                    th += vth/fabs(vth)*dth;
                }
            }
            else
            {
                double s = 0.0;
                double ds = 0.04;
                while(s <= (obs_dist + 0.1))
                {

                    tf::Transform d_tf , d_tf_new;
                    createTfFromXYTheta(s, 0, 0, d_tf);
                    d_tf_new = base_to_odom.inverse() * d_tf * base_to_odom;

                    double tmp_x = global_pose.getOrigin().getX() + d_tf_new.getOrigin().x();
                    double tmp_y = global_pose.getOrigin().getY() + d_tf_new.getOrigin().y();
                    double tmp_th = tf::getYaw(global_pose.getRotation());

                    double footprint_cost = world_model_->footprintCost(tmp_x, tmp_y, tmp_th,footprint_spec_,inscribed_radius_,circumscribed_radius_);
                    if(footprint_cost < 0)
                    {
                        ROS_INFO("stop move straight line when vx,%f,vth:%f,dist:%f in whole:%f",vx,vth,s,obs_dist);
                        //emergency stop with acc
						double actual_stop_dist = min_stop_dist_;
						ROS_INFO("last stop:%f,now:%f",last_stop_time.toSec(),ros::Time::now().toSec());
						if((last_stop_time + ros::Duration(5.0) > ros::Time::now()) && (min_stop_dist_ > 0.1))
						{
						  actual_stop_dist = min_stop_dist_ + 0.1;
						}
						ROS_INFO("actual_stop_dist:%f", actual_stop_dist);
						if(s > actual_stop_dist)
						{
							stopWithAcc(vx,vy,vth);
							return true;
						}
						else
						{
						    last_stop_time = ros::Time::now();
							vx = 0.0;
							vy = 0.0;
							vth = 0.0;
                            ROS_INFO("last stop time is now");
						}
                        return false;
                    }
                    s += ds;
                }
            }
            return true;
        }
        else if(TIME_METHOD == method)
        {
            if(fabs(vth) > 0.001)
            {
                double t = 0.0;
                double dt = 0.1;
                while(t < max_obstacle_time_)
                {
                    double theta = vth*t;
                    double x = sin(theta)*vx/vth;
                    double y = vx/vth * (1-cos(theta));

                    tf::Transform d_tf , d_tf_new;
                    createTfFromXYTheta(x, y, 0, d_tf);
                    d_tf_new = base_to_odom.inverse() * d_tf * base_to_odom;

                    double tmp_x = global_pose.getOrigin().getX() + d_tf_new.getOrigin().x();
                    double tmp_y = global_pose.getOrigin().getY() + d_tf_new.getOrigin().y();
                    double tmp_th = tf::getYaw(global_pose.getRotation()) + theta;
                    if (tmp_th >= M_PI)
                    {
                        tmp_th -= 2.0 * M_PI;
                    }
                    else if (tmp_th < -M_PI)
                    {
                        tmp_th += 2.0 * M_PI;
                    }

                    double footprint_cost = world_model_->footprintCost(tmp_x, tmp_y, tmp_th,footprint_spec_,inscribed_radius_,circumscribed_radius_);
                    if(footprint_cost < 0)
                    {
                        //emergency stop with acc
                        stopWithAcc(vx,vy,vth);
                        return false;
                    }
                    t += dt;
                }
            }
            else
            {
                double t = 0.0;
                double dt = 0.1;
                while(t < max_obstacle_time_)
                {
                    double x = t*vx;

                    tf::Transform d_tf , d_tf_new;
                    createTfFromXYTheta(x, 0, 0, d_tf);
                    d_tf_new = base_to_odom.inverse() * d_tf * base_to_odom;

                    double tmp_x = global_pose.getOrigin().getX() + d_tf_new.getOrigin().x();
                    double tmp_y = global_pose.getOrigin().getY() + d_tf_new.getOrigin().y();
                    double tmp_th = tf::getYaw(global_pose.getRotation());
                    if (tmp_th >= M_PI)
                    {
                        tmp_th -= 2.0 * M_PI;
                    }
                    else if (tmp_th < -M_PI)
                    {
                        tmp_th += 2.0 * M_PI;
                    }

                    double footprint_cost = world_model_->footprintCost(tmp_x, tmp_y, tmp_th,footprint_spec_,inscribed_radius_,circumscribed_radius_);
                    if(footprint_cost < 0)
                    {
                        //emergency stop with acc
                        stopWithAcc(vx,vy,vth);
                        return false;
                    }
                    t += dt;
                }
            }
            return true;
        }
    }
    void MrobotPlannerROS::createTfFromXYTheta(double x, double y, double theta, tf::Transform& t)
    {
        t.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        t.setRotation(q);
    }
    bool MrobotPlannerROS::calculateMaxVelScale(const tf::Stamped<tf::Pose>& global_pose,double& vx_scale,double& vy_scale,double& vth_scale)
    {
        //calculate obstacles effect
        //ROS_INFO("time 1");
        tf::StampedTransform base_to_odom;
        ros::Time t = ros::Time::now();
        try
        {
            tf_listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(0.2));
            tf_listener.lookupTransform ("base_link", "odom", ros::Time(0), base_to_odom);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Could not get initial transform from base to odom frame, %s", ex.what());
            return false;
        }
        //ROS_INFO("time 2");
        tf::Transform d_tf , d_tf_new;
        createTfFromXYTheta(obstacle_detect_center_, 0, 0, d_tf);
        d_tf_new = base_to_odom.inverse() * d_tf * base_to_odom;
        double obstacle_effect_dist = costmap_->getSizeInCellsX()*costmap_->getResolution()*0.5 - obstacle_detect_center_;
        if(obstacle_effect_dist > obstacle_detect_max_radius_)
        {
            obstacle_effect_dist = obstacle_detect_max_radius_;
        }
        //ROS_INFO("obstacle_effect_dist:%f",obstacle_effect_dist);
        int dist_size = obstacle_effect_dist/costmap_->getResolution();
        unsigned int current_x_index = 0;
        unsigned int current_y_index = 0;
        double current_x = d_tf_new.getOrigin().x()+global_pose.getOrigin().x();
        double current_y = d_tf_new.getOrigin().y()+global_pose.getOrigin().y();
        ROS_DEBUG("current x:%f,current y:%f,obstacle_effect_dist:%f,resolution:%f",current_x,current_y,obstacle_effect_dist,costmap_->getResolution());
        ROS_DEBUG("costmap origin x:%f, y:%f,sizex,y:%u * %u,",costmap_->getOriginX(),costmap_->getOriginY(),costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY());
        bool flag = costmap_->worldToMap(current_x,current_y,current_x_index,current_y_index);
        if(flag)
        {
            int act_x_min_index = std::max(0,(int)current_x_index-dist_size);
            int act_x_max_index = std::min(costmap_->getSizeInCellsX(),current_x_index+dist_size);

            int act_y_min_index = std::max(0,(int)current_y_index-dist_size);
            int act_y_max_index = std::min(costmap_->getSizeInCellsY(),current_y_index+dist_size);

            double weight_cost = 0.0;
            int sum_num = 0;
            for(int i=act_x_min_index;i<act_x_max_index;)
            {
                for(int j=act_y_min_index;j<act_y_max_index;)
                {
                    double act_index_x;
                    double act_index_y;
                    costmap_->mapToWorld(i,j,act_index_x,act_index_y);
                    double dist = sqrt(pow(act_index_x-current_x,2) + pow(act_index_y-current_y,2));
                    unsigned char cost = costmap_->getCost(i,j);
                    if(255 == cost)
                    {
                        cost = 0;
                    }
                    weight_cost += 0.5*(cos(dist/obstacle_effect_dist)+1)*cost;
                    //gavin modify 2018-1-30:weight_cost += fabs(1-dist/obstacle_effect_dist)*cost;
                    sum_num++;
                    j=j+3;
                }
                i=i+3;
            }
            double average_cost = weight_cost/sum_num;
            //ROS_INFO("average cost:%f",average_cost);
			if(average_cost >= 180)
            {
                vx_scale *= 0.2;
                vth_scale *= 0.3;
            }
            else if(average_cost >= 160)
            {
                vx_scale = -0.021 * average_cost + 3.959;
                vth_scale *= 0.7;
            }
            else if(average_cost >= 140)
            {
				vx_scale = -0.005 * average_cost + 1.395;
                vth_scale *= 0.8;
            }
            else if(average_cost >= 50)
            {
                vx_scale = -0.002 * average_cost + 0.978;
                vth_scale *= 0.9;
            }
            /*if(average_cost >= 180)
            {
                vx_scale *= 0.2;
                vth_scale *= 0.3;
            }
            else if(average_cost >= 160)
            {
                vx_scale = -0.026 * average_cost + 4.908;
                vth_scale *= 0.7;
            }
            else if(average_cost >= 140)
            {
                vx_scale *= 0.85;
                vth_scale *= 0.8;
            }
            else if(average_cost >= 50)
            {
                vx_scale *= 0.9;
                vth_scale *= 0.9;
            }*/
			ROS_INFO("average cost:%f, vx_scale:%f,vth_scale:%f",average_cost,vx_scale,vth_scale);
        }
        else
        {
            ROS_INFO("costmap world to map failed,use default scale");
            vx_scale *= 0.3;
            vth_scale *= 0.3;
        }
        return true;
    }

    bool MrobotPlannerROS::calculateActualMaxVel(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan,double& path_curvature)
    {
        double vx_scale = 1.0;
        double vy_scale = 1.0;
        double vth_scale = 1.0;

        calculateMaxVelScale(global_pose,vx_scale,vy_scale,vth_scale);

        //calculate curvature of path to decide max vel limit.
        //step 1:find nearst path point
        std::vector<geometry_msgs::PoseStamped>::iterator min_it;
        double min_dist = 10e8;
        path_curvature = 0.0;
        for(std::vector<geometry_msgs::PoseStamped>::iterator it=plan.begin();it != plan.end();it++)
        {
            double dx = global_pose.getOrigin().getX() - it->pose.position.x;
            double dy = global_pose.getOrigin().getY() - it->pose.position.y;
            if(sqrt(dx*dx + dy*dy) < min_dist)
            {
                min_it = it;
                min_dist = sqrt(dx*dx + dy*dy);
            }
        }

        //step 2:find next point distance over setup limit
        if(plan.end() - min_it >= 3)
        {
            //if we have 3 point at least.we could calculate curvature.
            double s = 0.0;
            std::vector<geometry_msgs::PoseStamped>::iterator second_it = min_it;
            std::vector<geometry_msgs::PoseStamped>::iterator third_it = min_it;
            for(std::vector<geometry_msgs::PoseStamped>::iterator it=min_it;it != plan.end();it++)
            {
                if(second_it != min_it)
                {
                    //have find second it;
                    s = sqrt(pow(it->pose.position.x - second_it->pose.position.x,2)+pow(it->pose.position.y - second_it->pose.position.y,2));
                    if(s >= curvature_dist_)
                    {
                        third_it = it;
                        break;
                    }
                }
                else
                {
                    //find second one;
                    s = sqrt(pow(it->pose.position.x - min_it->pose.position.x,2)+pow(it->pose.position.y - min_it->pose.position.y,2));
                    if(s >= curvature_dist_)
                    {
                        second_it = it;
                    }
                }
            }
            if((min_it != second_it) && (min_it != third_it))
            {
                //distance is too short,consider it as line
                double theta2 = atan2(third_it->pose.position.y - second_it->pose.position.y,third_it->pose.position.x - second_it->pose.position.x);
                double theta1 = atan2(second_it->pose.position.y - min_it->pose.position.y,second_it->pose.position.x - min_it->pose.position.x);
                double th = adjust_th(theta2-theta1);
                //ROS_INFO("calculate curvature th:%f",th);
                path_curvature = th/sqrt(pow(second_it->pose.position.x - min_it->pose.position.x,2)+pow(second_it->pose.position.y - min_it->pose.position.y,2));
            }
        }
        //ROS_INFO("path_curvature is:%f",path_curvature);


        ROS_DEBUG("final vx and vth scale:%f,%f",vx_scale,vth_scale);

        actual_max_vx_ = max_vx_*vx_scale;
        actual_max_vy_ = max_vy_*vy_scale;
        actual_max_vth_ = max_vth_*vth_scale;
        return true;
    }
  }


