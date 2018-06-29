#include "open_motion_planner/open_motion_planner.h"


OpenMotionPlanner::OpenMotionPlanner()
{
    ros::NodeHandle n("~");
    vel_pub_ = n.advertise<geometry_msgs::Twist>("/open_motion_vel",30);
    motion_status_pub_ = n.advertise<std_msgs::String>("/motion_status",30);
    robot_pose_sub_ = n.subscribe("/current_pose",20,&OpenMotionPlanner::RobotCurrentPose,this);
    scan_sub_ = n.subscribe("/scan_filtered",10,&OpenMotionPlanner::ScanCallback,this);

    //service of motion op
    op_srv = n.advertiseService("/open_motion_op",&OpenMotionPlanner::MotionOp,this);

    motion_param_.min_vx = 0.05;
    motion_param_.min_vy = 0;
    motion_param_.min_vth = 0.1;
    motion_param_.max_vx = 0.5;
    motion_param_.max_vy = 0.0;
    motion_param_.max_vth = 0.6;
    motion_param_.max_acc_x = 0.4;
    motion_param_.max_acc_y = 0.0;
    motion_param_.max_acc_th = 1.0;
    stop_limit_ = 0.5;

    action_.status = NAV_READY;
    vel_thread_ = new boost::thread(boost::bind(&OpenMotionPlanner::PlanVelThread, this));
}

OpenMotionPlanner::~OpenMotionPlanner()
{
    ROS_INFO("exit open motion planner");
}

void OpenMotionPlanner::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    latest_scan_ = *msg;
}

void OpenMotionPlanner::RobotCurrentPose(const nav_msgs::Odometry::ConstPtr &msg)
{
    robot_pose_ = *msg;
    //ROS_INFO("robot current pose:%f,%f,%f",robot_pose_.pose.pose.position.x,robot_pose_.pose.pose.position.y);
}

Json::Value OpenMotionPlanner::HandleCmd(Json::Value cmd)
{
    Json::Value res;
    
    tf::StampedTransform transform;

    res[MSG_SUB_NAME] = cmd[MSG_PUB_NAME];
    if(CMD_OPEN_MOTION_PAUSE == cmd[MSG_PUB_NAME].asString())
    {
        //pause;
        if((NAV_MOVING == action_.status) || (NAV_PAUSE == action_.status))
        {
            action_.status = NAV_PAUSE;
            res[MSG_ERROR_CODE] = 0;
            res[MSG_MSG] = "pause success";
        }
        else
        {
            res[MSG_ERROR_CODE] = 1;
            res[MSG_MSG] = "pause failed status is not right";
        }
    }
    else if(CMD_OPEN_MOTION_RESUME == cmd[MSG_PUB_NAME].asString())
    {
        //resume pause;
        if((NAV_MOVING == action_.status) || (NAV_PAUSE == action_.status))
        {
            action_.status = NAV_MOVING;
            res[MSG_ERROR_CODE] = 0;
            res[MSG_MSG] = "resume success";
        }
        else
        {
            res[MSG_ERROR_CODE] = 1;
            res[MSG_MSG] = "resume failed status is not right";
        }
    }
    else if(CMD_OPEN_MOTION_START == cmd[MSG_PUB_NAME].asString())
    {
        Json::Value data;
        data = cmd[MSG_DATA];
        //start a open motion;
        action_.update_info = true;
        action_.motion_type = (motion_type_e)(data["motion_type"].asInt());
        action_.time = data["time"].asDouble();
        action_.start_time = ros::Time::now();
        switch (action_.motion_type)
        {
            case VEL_MODE:
                action_.finished_flag = false;
                action_.vx = data["vx"].asDouble();
                action_.vy = data["vy"].asDouble();
                action_.vth = data["vth"].asDouble();
                action_.status = NAV_MOVING;
                res[MSG_ERROR_CODE] = 0;
                res[MSG_MSG] = "start success";
                break;
            case RELATIVE_POS_MODE:
                try{
                    listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
                }
                catch (tf::TransformException &ex) {
                  ROS_ERROR("%s",ex.what());
                  ros::Duration(1.0).sleep();
                  res[MSG_ERROR_CODE] = 1;
                  res[MSG_MSG] = "find odom between base_link transform failed";
                  break;
                }
                action_.ignore_xy_range = false;
                action_.finished_flag = false;
                action_.start_pose.x = transform.getOrigin().getX();
                action_.start_pose.y = transform.getOrigin().getY();
                action_.start_pose.theta = tf::getYaw(transform.getRotation());
                action_.target_x = data["target_x"].asDouble();
                action_.target_y = data["target_y"].asDouble();
                action_.target_th = data["target_th"].asDouble();
                action_.current_pose = action_.start_pose;
                action_.status = NAV_MOVING;
                res[MSG_ERROR_CODE] = 0;
                res[MSG_MSG] = "start success";
                break;
            case ABSOLUTE_POS_MODE:
                /*tf::StampedTransform transform;
                try{
                  listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
                }
                catch (tf::TransformException &ex) {
                  ROS_ERROR("%s",ex.what());
                  ros::Duration(1.0).sleep();
                  res[MSG_ERROR_CODE] = 1;
                  res[MSG_MSG] = "find map between base_link transform failed";
                  break;
                }
                action_.ignore_xy_range = false;
                action_.finished_flag = false;
                action_.start_pose.x = transform.getOrigin().getX();
                action_.start_pose.y = transform.getOrigin().getY();
                action_.start_pose.theta = tf::getYaw(transform.getRotation());
                action_.target_x = cmd["target_x"].asDouble();
                action_.target_y = cmd["target_y"].asDouble();
                action_.target_th = cmd["target_th"].asDouble();
                action_.current_pose = action_.start_pose;
                action_.status = NAV_MOVING;*/
                res[MSG_ERROR_CODE] = 1;
                res[MSG_MSG] = "unsupport motion now";
                break;
            default:
                ROS_ERROR("received a unknown motion type");
                break;
        }
        action_.update_info = false;
    }
    else if(CMD_OPEN_MOTION_CANCEL == cmd[MSG_PUB_NAME].asString())
    {
        geometry_msgs::Twist twist;
        //cancel a marker guide;
        if((NAV_MOVING == action_.status) || (NAV_PAUSE == action_.status))
        {
            action_.status = NAV_READY;
            action_.finished_flag = true;

            res[MSG_ERROR_CODE] = 0;
            res[MSG_MSG] = "cancel success";
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
            vel_pub_.publish(twist);
        }
        else
        {
            res[MSG_ERROR_CODE] = 1;
            res[MSG_MSG] = "cancel failed status is not right";
        }
    }
    return res;
}

bool OpenMotionPlanner::MotionOp(motion_planner::MotionOp::Request &req,motion_planner::MotionOp::Response &res)
{
    std::string test =req.cmd;
    Json::Reader reader;
    Json::Value value;
    Json::Value result;
    ROS_INFO("open motion:%s",test.c_str());
    if(reader.parse(test,value))
    {
       if(value[MSG_PUB_NAME].isNull())
       {
           ROS_INFO("pub_name value is empty");
       }
       else
       {
           //if it is not empty.
           result = HandleCmd(value);
           Json::StyledWriter fast;
           res.result = fast.write(result);
           res.error_code = 0;
           return true;
       }
    }
    res.result = "op failed!";
    res.error_code = 1;
    return false;
}

bool OpenMotionPlanner::CalRelativeMotion(move_action_t& action,move_param_t& param)
{
    if(ros::Time::now() < action_.start_time + ros::Duration(action_.time))
    {
        if(fabs(action.target_th) < EQUAL_VALUE)
        {
            action.cmd_vel.linear.x = action.target_x/action.time;
            action.cmd_vel.angular.z = 0.0;
        }
        else if(fabs(action.target_x) < EQUAL_VALUE)
        {
            action.cmd_vel.linear.x = 0.0;
            action.cmd_vel.angular.z = action.target_th/action.time;
        }
        else
        {
            double r = action.target_x /action.target_th;
            action.cmd_vel.linear.x = action.target_x/action.time;
            action.cmd_vel.angular.z = action.cmd_vel.linear.x / r;
        }
    }
    else
    {
        action_.status = NAV_READY;

        action.cmd_vel.linear.x = 0.0;
        action.cmd_vel.angular.z = 0.0;
        vel_pub_.publish(action.cmd_vel);

        Json::Value result;
        result["planner"] = MOVE_OPEN_MOTION;
        result["result"] = action_.status;
        Json::StyledWriter fast;
        std_msgs::String status;
        status.data = fast.write(result);
        motion_status_pub_.publish(status);
    }
}

bool OpenMotionPlanner::CalAbsoluteMotion(move_action_t& action,move_param_t& param)
{
    double abs_dx= action.target_x - action.current_pose.x;
    double abs_dy = action.target_y - action.current_pose.y;
    //double abs_dth = action.target_th - action.current_pose.theta;
    double th = atan2(abs_dy,abs_dx);
    double dist = sqrt(abs_dy * abs_dy + abs_dx * abs_dx);
    if(dist < 0.2)
    {
        action.ignore_xy_range = true;
    }
    if(!action.ignore_xy_range)
    {
        /*if(fabs(th - action.current_pose.theta) > 0.25)
        {
            action.vth = (action.current_pose.theta - th) * 0.5;
            action.vx = 0.0;
            if(fabs(action.vth)  > param.max_vth)
            {
                action.vth = param.max_vth*action.vth/fabs(action.vth);
            }
        }
        else
        {
            action.vx = sqrt(abs_dx*abs_dx + abs_dy*abs_dy)*0.5;
            if(action.vx > param.max_vx)
            {
                action.vx = param.max_vx;
            }
            action.vth =
        }
        double cvx = robot_pose_.twist.twist.linear.x;
        double cvth = robot_pose_.twist.twist.angular.z;
        double act_min_vx = std::min(cvx - 1.0*param.max_acc_x,0);
        double act_max_vx = std::max(cvx + 1.0*param.max_acc_x,param.max_vx);
        double act_min_vth = std::min(cvth - 1.0*param.max_acc_th,-param.max_vth);
        double act_max_vth = std::max(cvth + 1.0*param.max_acc_th,param.max_vth);
        for(double vx = act_min_vx;vx < act_max_vx;vx+=0.05)
        {
            for(double vth = act_min_vth;vth < act_max_vth;vth+=0.02)
            {
                if(fabs(vth) < EQUAL_VALUE)
                {
                    //vth = 0.0
                }
                else
                {
                    double r = vx/vth;
                }
            }
        }*/
        action.vth = 0.0;
        action.vx = 0.0;
    }
    else
    {
        if(fabs(action.target_th - action.current_pose.theta) < 0.1)
        {
            action_.status = NAV_READY;

            action.cmd_vel.linear.x = 0.0;
            action.cmd_vel.angular.z = 0.0;
            vel_pub_.publish(action.cmd_vel);

            Json::Value result;
            result["planner"] = MOVE_OPEN_MOTION;
            result["result"] = action_.status;
            Json::StyledWriter fast;
            std_msgs::String status;
            status.data = fast.write(result);
            motion_status_pub_.publish(status);
        }
        else
        {
            action.vth = (action.target_th - action.current_pose.theta) * 0.5;
            action.vx = 0.0;
            if(fabs(action.vth)  > param.max_vth)
            {
                action.vth = param.max_vth*action.vth/fabs(action.vth);
            }
        }
    }
}

void OpenMotionPlanner::check_obstacle_vel(geometry_msgs::Twist &twist)
{
    /*for(int i=0; i < latest_scan_.ranges.size();i++)
    {

    }
    if(twist.linear.x >= 0)
    {
        if(fabs(twist.angular.z) < 0.1)
        {
            //line move
            if((laser scan < stop_limit_)&&(laser and sonar < stop_limit_))
            {
                twist.linear.x = 0;
                twist.angular.z = 0;
            }
        }
        else if(twist.angular.z > 0.0)
        {
            if(left have obstalce in stop_limit_)
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
            }
        }
        else
        {
            if(right have obstacle in stop_limit_)
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
            }
        }
    }
    else
    {
        if(back have obstacles)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
    }*/
    return;
}

void OpenMotionPlanner::PlanVelThread()
{
    geometry_msgs::Twist twist;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Rate r(20);
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    while(ros::ok())
    {
        if(NAV_MOVING == action_.status)
        {
            switch (action_.motion_type)
            {
            case VEL_MODE:
                if(ros::Time::now() < action_.start_time + ros::Duration(action_.time))
                {
                    twist.linear.x = action_.vx;
                    twist.angular.z = action_.vth;
                }
                else
                {
                    action_.status = NAV_READY;

                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;                    
                    vel_pub_.publish(twist);

                    Json::Value result;
                    result["planner"] = MOVE_OPEN_MOTION;
                    result["result"] = action_.status;
                    Json::StyledWriter fast;
                    std_msgs::String status;
                    status.data = fast.write(result);
                    motion_status_pub_.publish(status);
                }
                break;
            case RELATIVE_POS_MODE:
                ROS_INFO("relative pos mode");
                try{
                  listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
                }
                catch (tf::TransformException &ex) {
                  ROS_ERROR("%s",ex.what());
                  ros::Duration(1.0).sleep();
                  break;
                }
                action_.current_pose.x = transform.getOrigin().getX();
                action_.current_pose.y = transform.getOrigin().getY();
                action_.current_pose.theta = tf::getYaw(transform.getRotation());
                //calculate cmd_vel
                CalRelativeMotion(action_,motion_param_);
                twist = action_.cmd_vel;
                break;
            case ABSOLUTE_POS_MODE:
                ROS_INFO("absolute pos mode");
                try{
                  listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
                }
                catch (tf::TransformException &ex) {
                  ROS_ERROR("%s",ex.what());
                  ros::Duration(1.0).sleep();
                  break;
                }
                action_.current_pose.x = transform.getOrigin().getX();
                action_.current_pose.y = transform.getOrigin().getY();
                action_.current_pose.theta = tf::getYaw(transform.getRotation());
                //calculate cmd_vel
                CalAbsoluteMotion(action_,motion_param_);
                twist = action_.cmd_vel;
                break;
            default:
                break;
            }
            check_obstacle_vel(twist);
        }
        else if(NAV_PAUSE == action_.status)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }

        if(NAV_READY != action_.status)
        {
            vel_pub_.publish(twist);
        }
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_motion_planner");

    OpenMotionPlanner planner;

    ros::Rate r(20);
    while(ros::ok())
    {
        if(!(ros::master::check()))
        {
            ROS_ERROR("master check failed!");
        }
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
