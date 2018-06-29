#include "uwb_location/uwb_location_node.h"

UwbLocationNode::UwbLocationNode()
{
    ros::NodeHandle priv_nh("~");
    ros::NodeHandle nh;
    if(!priv_nh.getParam("update_rate", update_rate))
        update_rate = 10.0;
    // update pose when current pose has max distance with uwb
    if(!priv_nh.getParam("max_offset_with_currentpose", max_offset_with_currentpose))
        max_offset_with_currentpose = 2.5;
    // uwb switch
    if(!priv_nh.getParam("is_use_uwb", is_use_uwb))
        is_use_uwb = false;
    // uwb data is ok when it is not change max in one period
    if(!priv_nh.getParam("max_diff_range_self", max_diff_range_self))
        max_diff_range_self = 0.1;
     // uwb data is ok when it changed distance comprare with scan odom in timer_duration period
    if(!priv_nh.getParam("timer_max_offset", timer_max_offset))
        timer_max_offset = 0.1;
    // uwb compare with scan period
    if(!priv_nh.getParam("timer_duration", timer_duration))
        timer_duration = 2.0;

    if(!priv_nh.getParam("cov_x", cov_x))
        cov_x = 0.25;
    if(!priv_nh.getParam("cov_y", cov_y))
        cov_y = 0.25;
    if(!priv_nh.getParam("cov_th", cov_th))
        cov_th = 9.68;
    // is use frame map not odom
    if(!priv_nh.getParam("is_use_map", is_use_map))
        is_use_map = true;
    // compute th for tf after move min_distance
    if(!priv_nh.getParam("update_min_distance", update_min_distance))
        update_min_distance = 2.5;

	last_unread_bytes = 0;
    count_compute = 0;
    first_uwb_flag = true;
    timer_first = true;
    uwb_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("uwb_pose", 5);
	uwb_odom_pub = nh.advertise<nav_msgs::Odometry>("/uwb_odom",100);
	uwb_odom_origin_pub = nh.advertise<nav_msgs::Odometry>("/uwb_odom_origin",100);
    uwb_status_pub = nh.advertise<std_msgs::String>("/uwb_status",100);
    initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);
    uwb_anchors_pub = nh.advertise<uwb_location::UwbAnchors>("/uwb_anchors",100);
    current_pose_sub = nh.subscribe("/current_pose", 1, &UwbLocationNode::current_pose_received, this);
    odom_sub = nh.subscribe("/scan_odom", 1, &UwbLocationNode::odom_received, this);
    nomotion_client = nh.serviceClient<std_srvs::Empty>("/request_nomotion_update");
    query_uwb_client = nh.serviceClient<map_server::query_uwb>("/map_server_mrobot/query_uwb_points");
    boost::thread rcv_thread(boost::bind(&UwbLocationNode::uwb_driver_thread, this));
    //boost::thread compute_thread(boost::bind(&UwbLocationNode::uwb_compute_thread, this));
    uwb_timer = nh.createTimer(ros::Duration(timer_duration),&UwbLocationNode::timer_callback,this);

    for(int num =0; num<ANCHOR_NUM;num++)
    {
        anchors[num].id = 0;
    }

}

UwbLocationNode::~UwbLocationNode()
{

}

/*
*  is uwb data ok when it changed distance comprare with scan odom
*/
void UwbLocationNode::timer_callback(const ros::TimerEvent &e)
{
    if(!is_use_uwb)
    {
        return;
    }
    timer_odom_pose = odom_pose;
    timer_uwb_pose = uwb_pose;
    if(timer_first)
    { 
        timer_latest_odom_pose = odom_pose;
        timer_latest_uwb_pose = uwb_pose;
        timer_first = false;
    }
    else
    {
        timer_offset = hypot(timer_odom_pose.position.x-timer_latest_odom_pose.position.x,
                             timer_odom_pose.position.y-timer_latest_odom_pose.position.y)
                       -hypot(timer_uwb_pose.position.x-timer_latest_uwb_pose.position.x,
                              timer_uwb_pose.position.y-timer_latest_uwb_pose.position.y);
        is_uwb_good_compare_scan = (std::abs(timer_offset) <= timer_max_offset);
        ROS_DEBUG("timer offset timer_offset %.3f",timer_offset);
        timer_latest_odom_pose = timer_odom_pose;
        timer_latest_uwb_pose = timer_uwb_pose;
    }
}

bool less_sort(map_server::point a,map_server::point b) 
{ 
    return (a.index<b.index); 
}

/*
* first get vector and sort index , get usefull range for uwb 
* next is current pose in range
*/
bool UwbLocationNode::is_current_pose_in_uwb(geometry_msgs::Pose pose)
{
    bool rlt = false;
    map_server::query_uwb srv;
    query_uwb_client.call(srv);
    vec = srv.response.uwb_points;
    int nCross = 0;
    int nCount;
    if(0 != vec.size())
    {
        //sort dsc
        sort(vec.begin(),vec.end(),less_sort);
        nCount = vec.size();
        //current pose in range when nCross is odd ,offside out of range
        for (int i = 0; i < nCount; i++)
        {  
            map_server::point p1 = vec[i];  
            map_server::point p2 = vec[(i + 1) % nCount];  
            if (p1.y == p2.y)  
            {
                continue;  
            }  
            if (pose.position.y < std::min(p1.y, p2.y))  
            {  
                continue;  
            }  
            if (pose.position.y >= std::max(p1.y, p2.y))  
            {
                continue;  
            }  
            double x = (double)(pose.position.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;  
            if (x > pose.position.x){  
                nCross++; 
            }  
        }
        rlt = (nCross % 2 == 1);
    }
    ROS_DEBUG("uwb current pose in range %d",nCross);
    return rlt;

}

/*
* get current pose and is in uwb range and data is ok
* if current pose has max distance with uwb then update robot pose
*/
void UwbLocationNode::current_pose_received(const nav_msgs::Odometry& msg)
{
    boost::mutex::scoped_lock(mutex_);
    robot_current_pose = msg.pose.pose;
    if(is_use_uwb && is_current_pose_in_uwb(robot_current_pose))
    {
        double distance;
        std_srvs::Empty srv;
        distance = hypot(robot_current_pose.position.x - uwb_pose.position.x,
                            robot_current_pose.position.y - uwb_pose.position.y);
        ROS_DEBUG("uwb offset with current pose:%.3f(%.3f)",distance,max_offset_with_currentpose);
        // if current pose has max distance with uwb then update robot pose
        if(distance >= max_offset_with_currentpose && is_uwb_good_self && is_uwb_good_compare_scan)
        {
            initial_pose_pub.publish(uwb_pose_with_cov);
            nomotion_client.call(srv);
        }
    }
}

/*
* get scan odom data
*/
void UwbLocationNode::odom_received(const nav_msgs::Odometry& msg)
{
    boost::mutex::scoped_lock(mutex_);
    if(!is_use_uwb)
    {
        return;
    }
    odom_pose = msg.pose.pose;
}

/*
* compute th for tf after move min_distance which get by vector 
*/
void UwbLocationNode::uwb_compute_thread()
{
    ros::Rate r(update_rate);

    double odom_dx, odom_dy, current_dx, current_dy, uwb_dx, uwb_dy;
    double x1, x2, y1, y2;
    double vector_inter_mult;
    double vector_mo_mult;
    double vector_across_mult;
    double uwb_map_th_tmp;
    latest_odom_pose = odom_pose;
    latest_uwb_pose = uwb_pose; 
    is_compute_th = true;
    compute_th_ok = false;
    tf::Quaternion uwb_quaternion;
    while(ros::ok())
    {
        // if use map then can not compute th
        if(is_use_uwb && !is_use_map)
        {
            // compute odom and uwb tf :first_uwb_pose x,y,uwb_map_th
            odom_dx = odom_pose.position.x - latest_odom_pose.position.x;
            odom_dy = odom_pose.position.y - latest_odom_pose.position.y;
            if(((odom_dx*odom_dx + odom_dy*odom_dy) >= update_min_distance*update_min_distance) && is_compute_th)
            {
                x1 = latest_odom_pose.position.x - odom_pose.position.x;
                y1 = latest_odom_pose.position.y - odom_pose.position.y;
                x2 = latest_uwb_pose.position.x - uwb_pose.position.x;
                y2 = latest_uwb_pose.position.y - uwb_pose.position.y;
                vector_inter_mult = x1 * x2 + y1 * y2;
                vector_across_mult = x1 * y2 - x2 * y1;   
                vector_mo_mult = hypot(x1,y1) *  hypot(x2,y2);           
                uwb_map_th_tmp = (vector_across_mult>=0)?
                                -std::acos(vector_inter_mult/vector_mo_mult):
                                std::acos(vector_inter_mult/vector_mo_mult);
                if(!std::isnan(uwb_map_th_tmp))
                {
                    uwb_map_ths[count_compute] = uwb_map_th_tmp;
                    count_compute++;
                    // get 2 times for more better th
                    if(count_compute == 2)
                    {
                        uwb_map_th = (uwb_map_ths[0]+uwb_map_ths[1])/2;
                        transform_uwb.setOrigin(tf::Vector3(first_uwb_pose.position.x, 
                                                            first_uwb_pose.position.y, 
                                                            first_uwb_pose.position.z));
                        uwb_quaternion.setRPY(0, 0, uwb_map_th);
                        transform_uwb.setRotation(uwb_quaternion);
                        is_compute_th = false;
                        compute_th_ok = true;	
                    }
                }
                //ROS_INFO("uwb uwb_map_th:%.3f",uwb_map_th/3.14*180);
                latest_odom_pose = odom_pose;
                latest_uwb_pose = uwb_pose;
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}

/*
* pub uwb_odom uwb_pose
*/
void UwbLocationNode::pub_uwb_info()
{
    tf::Pose pose_old, pose_new;
    geometry_msgs::Pose uwb_odom_pose;
    if(compute_th_ok && !is_use_map)
    {
        tf::poseMsgToTF(uwb_pose, pose_old);
        pose_new = transform_uwb * pose_old;
        tf::poseTFToMsg(pose_new,uwb_odom_pose);

        //pub uwb_odom
        uwb_odom.header.frame_id = "odom";
        uwb_odom.header.stamp = ros::Time::now();
        uwb_odom.pose.pose = uwb_odom_pose;
        uwb_odom.pose.covariance = boost::assign::list_of
                (cov_x) (0)  (0)  (0)  (0)  (0)
                (0)  (cov_y) (0)  (0)  (0)  (0)
                (0)  (0)  (99999) (0)  (0)  (0)
                (0)  (0)  (0)  (99999) (0)  (0)
                (0)  (0)  (0)  (0)  (99999) (0)
                (0)  (0)  (0)  (0)  (0)  (99999);
        uwb_odom.child_frame_id = "base_link";
        uwb_odom.twist.twist.linear.x = 0;
        uwb_odom.twist.twist.linear.y = 0.0;
        uwb_odom.twist.twist.angular.z = 0;
        uwb_odom_pub.publish(uwb_odom);

        //pub uwb_pose
        uwb_pose_with_cov.header.frame_id = "odom";
        uwb_pose_with_cov.header.stamp = ros::Time::now();
        uwb_pose_with_cov.pose.pose = uwb_odom_pose;
        uwb_pose_with_cov.pose.covariance = boost::assign::list_of
                (cov_x) (0)  (0)  (0)  (0)  (0)
                (0)  (cov_y) (0)  (0)  (0)  (0)
                (0)  (0)  (99999) (0)  (0)  (0)
                (0)  (0)  (0)  (99999) (0)  (0)
                (0)  (0)  (0)  (0)  (99999) (0)
                (0)  (0)  (0)  (0)  (0)  (99999);
	    uwb_pose_pub.publish(uwb_pose_with_cov);

	    //pub uwb_odom_origin
	    uwb_odom.header.frame_id = "odom";
        uwb_odom.header.stamp = ros::Time::now();
        uwb_odom.pose.pose = uwb_pose;
        uwb_odom.pose.covariance = boost::assign::list_of
                (cov_x) (0)  (0)  (0)  (0)  (0)
                (0)  (cov_y) (0)  (0)  (0)  (0)
                (0)  (0)  (99999) (0)  (0)  (0)
                (0)  (0)  (0)  (99999) (0)  (0)
                (0)  (0)  (0)  (0)  (99999) (0)
                (0)  (0)  (0)  (0)  (0)  (99999);
        uwb_odom.child_frame_id = "base_link";
        uwb_odom.twist.twist.linear.x = 0;
        uwb_odom.twist.twist.linear.y = 0.0;
        uwb_odom.twist.twist.angular.z = 0;
        uwb_odom_origin_pub.publish(uwb_odom);
    }
    if(is_use_map)
    {
        //pub uwb_pose
        uwb_pose_with_cov.header.frame_id = "map";
        uwb_pose_with_cov.header.stamp = ros::Time::now();
        uwb_pose_with_cov.pose.pose = uwb_pose;
        uwb_pose_with_cov.pose.covariance = boost::assign::list_of
                (cov_x) (0)  (0)  (0)  (0)  (0)
                (0)  (cov_y) (0)  (0)  (0)  (0)
                (0)  (0)  (0) (0)  (0)  (0)
                (0)  (0)  (0)  (0) (0)  (0)
                (0)  (0)  (0)  (0)  (0) (0)
                (0)  (0)  (0)  (0)  (0)  (cov_th);
	    uwb_pose_pub.publish(uwb_pose_with_cov);
        //pub uwb_odom
		uwb_odom.header.frame_id = "map";
        uwb_odom.header.stamp = ros::Time::now();
        uwb_odom.pose.pose = uwb_pose;
        uwb_odom.pose.covariance = boost::assign::list_of
                (cov_x) (0)  (0)  (0)  (0)  (0)
                (0)  (cov_y) (0)  (0)  (0)  (0)
                (0)  (0)  (0) (0)  (0)  (0)
                (0)  (0)  (0)  (0) (0)  (0)
                (0)  (0)  (0)  (0)  (0) (0)
                (0)  (0)  (0)  (0)  (0)  (cov_th);
        uwb_odom.child_frame_id = "base_link";
        uwb_odom.twist.twist.linear.x = 0;
        uwb_odom.twist.twist.linear.y = 0.0;
        uwb_odom.twist.twist.angular.z = 0;
        uwb_odom_pub.publish(uwb_odom);
    }
}

/*
* set uwb pose from usb data
*/
void UwbLocationNode::set_uwb_pose(double x, double y, double z)
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
    uwb_pose.position.x = x;
    uwb_pose.position.y = y;
    uwb_pose.position.z = z;
    uwb_pose.orientation = odom_quat;
    ROS_DEBUG("uwb x,y,z:%f,%f,%f",x,y,z);
	if(first_uwb_flag)
	{
		first_uwb_pose.position.x = x;
		first_uwb_pose.position.y = y;
		first_uwb_pose.position.z = z;
		first_uwb_pose.orientation = odom_quat;
		first_uwb_flag = false;
        pre_uwb_pose = first_uwb_pose;
        return;
    }
    // uwb data is ok when it is not change too large in one period
    diff_range = hypot(uwb_pose.position.x-pre_uwb_pose.position.x, 
                                uwb_pose.position.y-pre_uwb_pose.position.y);
	ROS_DEBUG("uwb diff_range:%.3f",diff_range);
    pre_uwb_pose = uwb_pose;
    
    is_uwb_good_self = (diff_range <= max_diff_range_self);
    std_msgs::String rlt;
    rlt.data = (is_uwb_good_self && is_uwb_good_compare_scan)?"good":"bad";
    uwb_status_pub.publish(rlt);
}

/*
* update_system_state
*/
void UwbLocationNode::update_system_state()
{
    static int last_file_flag = 0;
    struct stat file_info;
    int i = 0;

    switch(uwb_sys.com_state)
    {
        case COM_OPENING:
            uwb_sys.work_normal = 0;
            i = stat(uwb_sys.dev,&file_info);
            if(-1 == i)
            {
                if(-1 != last_file_flag)
                {
                    ROS_DEBUG("uwb com device does not exist\n");
                }
                last_file_flag = i;
                return;
            }
            last_file_flag = i;
            uwb_sys.com_device = uwbSerial.open_com_device(uwb_sys.dev);
            if(-1 != uwb_sys.com_device)
            {
                uwb_sys.com_state = COM_CHECK_VERSION;
                uwb_sys.work_normal = 1;
                ROS_DEBUG("open uwb com success!");
            }
            else
            {
                ROS_DEBUG("open uwb com device failed");
                return;
            }
            uwbSerial.set_speed(uwb_sys.com_device,115200);
            uwbSerial.set_parity(uwb_sys.com_device,8,1,'N');

            break;

        case COM_CHECK_VERSION:

            uwb_sys.com_state = COM_RUN_OK;
            break;

        case COM_RUN_OK:
            //ROS_DEBUG("COM_RUN_OK!!");
            break;

        case COM_CLOSING:
            close(uwb_sys.com_device);
            uwb_sys.com_state = COM_OPENING;
            uwb_sys.com_rssi = 0;
            uwb_sys.uwb_rssi = 0;
            uwb_sys.work_normal = 0;
            ROS_DEBUG("close com device:%d\n",uwb_sys.com_device);
            break;
        default:
            break;
    }

    return;
}

/*
* handle_receive_data
*/
int UwbLocationNode::handle_receive_data()
{
    int nread = 0;
    int i = 0;
    int j = 0;
    int data_Len = 0;
    unsigned char recv_buf[BUF_LEN] = {0};
    unsigned char recv_buf_complete[BUF_LEN] = {0};

    struct stat file_info;

    if(COM_RUN_OK != uwb_sys.com_state && COM_CHECK_VERSION != uwb_sys.com_state)
    {
        ROS_DEBUG("uwb_handle_receive_data: com_state != COM_RUN_OK && COM_CHECK_VERSION");
        return -1;
    }

    if(0 != last_unread_bytes)
    {
        for(j=0;j<last_unread_bytes;j++)
        {
            recv_buf_complete [j] = recv_buf_last[j];
        }
    }
    if((nread = read(uwb_sys.com_device, recv_buf, BUF_LEN))>0)
    {
        memcpy(recv_buf_complete+last_unread_bytes,recv_buf,nread);
        data_Len = last_unread_bytes + nread;
        last_unread_bytes = 0;
        /*for(j=0;j<data_Len;j++)
        {
            ROS_INFO("uwb data :%02x",recv_buf_complete [j]);
        }*/
        while(i<data_Len)
        {
            if(0x57 == recv_buf_complete [i] && 0x58 == recv_buf_complete[i+1])
            {
                if(0x40 == recv_buf_complete[i+2] && 0x0c == recv_buf_complete[i+3])
                {
                    if(i+18 <= data_Len)
                    {
                        unsigned short sum=0 , check_sum = 0;
                        for (j=0; j<16; j++)
                        {
                            sum += recv_buf_complete[i+j];
                        }
                        *(char *)&check_sum = recv_buf_complete[i+16];
                        *((char *)&check_sum+1) = recv_buf_complete[i+17];

                        if(sum == check_sum)
                        {
                            short int x = 0;
                            *(char *)&x = recv_buf_complete[i+8];
                            *((char *)&x+1) = recv_buf_complete[i+9];
                            short int y = 0;
                            *(char *)&y = recv_buf_complete[i+10];
                            *((char *)&y+1) = recv_buf_complete[i+11];
                            short int z = 0;
                            *(char *)&z = recv_buf_complete[i+12];
                            *((char *)&z+1) = recv_buf_complete[i+13];
                            //ROS_INFO("uwb tag pose x:%.3f, y:%.3f, z:%.3f",x/100.0,y/100.0,z/100.0);
                            set_uwb_pose(x/100.0,y/100.0,z/100.0); 
						    pub_uwb_info();                   
                            i  = i + 18;
                        }
                        else
                        {
                            ROS_ERROR("sum != check sum");
                            i++;
                        }
                    }
                    else
                    {
                        last_unread_bytes = data_Len - i;
                        for(j=0;j<last_unread_bytes;j++)
                        {
                            recv_buf_last[j] = recv_buf_complete[i+j];
                        }
                        break;
                    }
                }
                else if(0x10 == recv_buf_complete[i+2] && 0x20 == recv_buf_complete[i+3])
                {
                    if(i+38 <= data_Len)
                    {
                        unsigned short sum=0 , check_sum = 0;
                        for (j=0; j<36; j++)
                        {
                            sum += recv_buf_complete[i+j];
                        }
                        *(char *)&check_sum = recv_buf_complete[i+36];
                        *((char *)&check_sum+1) = recv_buf_complete[i+37];

                        if(sum == check_sum)
                        {
                            short int id = 0;
                            *(char *)&id = recv_buf_complete[i+14];
                            *((char *)&id+1) = recv_buf_complete[i+15];
                            short int distance = 0;
                            *(char *)&distance = recv_buf_complete[i+22];
                            *((char *)&distance+1) = recv_buf_complete[i+23];
                            //ROS_INFO("uwb anchor id:%04x, distance:%.2f",id,distance/100.0);
                            for(int num =0; num<ANCHOR_NUM;num++)
                            {
                                if(anchors[num].id == id)
                                {
                                    anchors[num].distance = distance/100.0;
                                    break;
                                }
                                else if(anchors[num].id == 0)
                                {
                                    anchors[num].id = id;
                                    anchors[num].distance = distance/100.0;
                                    break;
                                }
                                continue;
                            }
                            i  = i + 38;
                        }
                        else
                        {
                            ROS_ERROR("sum != check sum");
                            i++;
                        }
                    }
                    else
                    {
                        last_unread_bytes = data_Len - i;
                        for(j=0;j<last_unread_bytes;j++)
                        {
                            recv_buf_last[j] = recv_buf_complete[i+j];
                        }
                        break;
                    }
                }
                else
                {
                    i++;
                }    
            }
            else
            {
                i++;
            }    
        }
        uwb_anchors.header.frame_id = "map";
        uwb_anchors.header.stamp = ros::Time::now();
        uwb_location::Anchor anchor;
        //std::string temp;
        for(j=0;j<ANCHOR_NUM;j++)
        {
            anchor.id = anchors[j].id;
            anchor.distance = anchors[j].distance;
            uwb_anchors.anchors.push_back(anchor);
            //std::stringstream ss;
            //ss<<anchor.id<<" "<<anchor.distance<<" ";
            //temp+=ss.str();
            //ROS_INFO("uwb anchor id:%04x, distance:%.2f",anchors[j].id,anchors[j].distance);
        }
        uwb_anchors_pub.publish(uwb_anchors);
        uwb_anchors.anchors.clear();
        //uwb_file<<uwb_pose.position.x<<" "<<uwb_pose.position.y<<" "<<uwb_pose.position.z<<" "<<temp<< std::endl;
    }
    else
    {
        i = stat(uwb_sys.dev,&file_info);
        if(-1 == i)
        {
            uwb_sys.com_state = COM_CLOSING;
        }
    }
    return 0;
}

/*
* get uwb data
*/
void UwbLocationNode::uwb_driver_thread()
{
    ros::Rate r(update_rate);
    uwb_sys.com_state = COM_OPENING;
    char com_device_path[]="/dev/ros/uwb";
    memcpy(uwb_sys.dev,com_device_path,sizeof(com_device_path));
    //uwb_file.open("/home/robot/uwb.txt");
    while(ros::ok())
    {
        if(is_use_uwb)
        {
            update_system_state();
            handle_receive_data();
        }
        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_location_node");
    UwbLocationNode uwbLocationNode;
	ros::spin();
    return 0;
}