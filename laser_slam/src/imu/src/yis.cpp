#include "imu/yis.h"

static yis_sys_t yis_sys;
static ros::Publisher imu_pub;
static ros::Publisher mag_pub;
volatile float imu_odom_x_;
volatile float imu_odom_y_;

static void pub_imu(yis_sys_t &sys)
{
    sensor_msgs::Imu imuData;
    imuData.header.frame_id = "base_link";
    imuData.header.stamp = ros::Time::now();

    imuData.orientation.w = sys.q0;
    imuData.orientation.x = sys.q1;
    imuData.orientation.y = sys.q2;
    imuData.orientation.z = sys.q3;

    imuData.angular_velocity.x = PI * sys.wx / 180;
    imuData.angular_velocity.y = PI * sys.wy / 180;
    imuData.angular_velocity.z = PI * sys.wz / 180;

    imuData.linear_acceleration.x = sys.fax;
    imuData.linear_acceleration.y = sys.fay;
    imuData.linear_acceleration.z = sys.faz;

    imuData.orientation_covariance = {0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0};

    imuData.angular_velocity_covariance = {0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0, 
                                            0.0, 0.0, 0.0};

    imuData.linear_acceleration_covariance = {0.0, 0.0, 0.0, 
                                                0.0, 0.0, 0.0, 
                                                0.0, 0.0, 0.0};

    imu_pub.publish(imuData);
}

static void pub_mag(yis_sys_t &sys)
{
    sensor_msgs::MagneticField magData;
    magData.header.frame_id = "base_link";
    magData.header.stamp = ros::Time::now();
    magData.magnetic_field.x = sys.mx * 1e-7;
    magData.magnetic_field.y = sys.my * 1e-7;
    magData.magnetic_field.z = sys.mz * 1e-7;
    magData.magnetic_field_covariance = {0.0, 0.0, 0.0, 
                                         0.0, 0.0, 0.0, 
                                         0.0, 0.0, 0.0};
    mag_pub.publish(magData);
}

static double cal_by_buf(unsigned char buf[], int position, float factor)
{
    int a0 = buf[position];
    int a1 = buf[position + 1] << 8;
    int a2 = buf[position + 2] << 16;
    int a3 = buf[position + 3] << 24;
    return (a0 + a1 + a2 + a3) * factor;
}

static void handle_receive_data(yis_sys_t &sys)
{
    int nread  = 0;
    unsigned char buf[BUF_LEN] = {0};

    if((nread = read(sys.com_device, buf, BUF_LEN)) > 0)
    {
        if(nread >= 127 && buf[0] == 0x59 && buf[1] == 0x49 && buf[2] == 0x53) 
        {
            int position = 0;
            if(buf[6] == 0x10 && buf[7] == 12)
            {
                position = 8;
                sys.ax = cal_by_buf(buf, position, 1e-6);
                sys.ay = cal_by_buf(buf, position + 4, 1e-6);
                sys.az = cal_by_buf(buf, position + 8, 1e-6);
            }
            else
            {
                ROS_ERROR("fail to get acceleration return");
                return;
            }
            
            if(buf[20] == 0x11 && buf[21] == 12)
            {
                position = 22;
                sys.fax = cal_by_buf(buf, position, 1e-6);
                sys.fay = cal_by_buf(buf, position + 4, 1e-6);
                sys.faz = cal_by_buf(buf, position + 8, 1e-6);
            }
            else
            {
                ROS_ERROR("fail to get free acceleration return");
                return;
            }

            if(buf[34] == 0x12 && buf[35] == 12)
            {
                position = 36;
                sys.dvx = cal_by_buf(buf, position, 1e-6);
                sys.dvy = cal_by_buf(buf, position + 4, 1e-6);
                sys.dvz = cal_by_buf(buf, position + 8, 1e-6);
            }
            else
            {
                ROS_ERROR("fail to get delta velocity return");
                return;
            }

            if(buf[48] == 0x20 && buf[49] == 12)
            {
                position = 50;
                sys.wx = cal_by_buf(buf, position, 1e-6);
                sys.wy = cal_by_buf(buf, position + 4, 1e-6);
                sys.wz = cal_by_buf(buf, position + 8, 1e-6);
            }
            else
            {
                ROS_ERROR("fail to get angular velocity return");
                return;
            }

            if(buf[62] == 0x30 && buf[63] == 12)
            {
                position = 64;
                sys.mx = cal_by_buf(buf, position, 1e-3);
                sys.my = cal_by_buf(buf, position + 4, 1e-3);
                sys.mz = cal_by_buf(buf, position + 8, 1e-3);
            }
            else
            {
                ROS_ERROR("fail to get magnetic field return");
                return;
            }

            if(buf[76] == 0x40 && buf[77] == 12)
            {
                position = 78;
                sys.pitch = cal_by_buf(buf, position, 1e-6);
                sys.roll = cal_by_buf(buf, position + 4, 1e-6);
                sys.yaw = cal_by_buf(buf, position + 8, 1e-6);
            }
            else
            {
                ROS_ERROR("fail to get euler angles return");
                return;
            }

            if(buf[90] == 0x41 && buf[91] == 16)
            {
                position = 92;
                sys.q0 = cal_by_buf(buf, position, 1e-6);
                sys.q1 = cal_by_buf(buf, position + 4, 1e-6);
                sys.q2 = cal_by_buf(buf, position + 8, 1e-6);
                sys.q3 = cal_by_buf(buf, position + 12, 1e-6);
            }
            else
            {
                ROS_ERROR("fail to quaternion return");
                return;
            }

            if(buf[108] == 0x42 && buf[109] == 16)
            {
                position = 110;
                sys.dq0 = cal_by_buf(buf, position, 1e-6);
                sys.dq1 = cal_by_buf(buf, position + 4, 1e-6);
                sys.dq2 = cal_by_buf(buf, position + 8, 1e-6);
                sys.dq3 = cal_by_buf(buf, position + 12, 1e-6);
            }
            else
            {
                ROS_ERROR("fail to delta quaternion return");
                return;
            }
            pub_imu(sys);
            pub_mag(sys);
        }
    }
}

// void myOdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
// {
//   imu_odom_x_ = odom->pose.pose.position.x;
//   imu_odom_y_ = odom->pose.pose.position.y;
// }

// void pub_imu_odom(ros::Publisher &imu_odom_pub)
// {
//   nav_msgs::Odometry odom;

//   odom.header.frame_id = "odom";
//   odom.header.stamp = ros::Time::now();
//   odom.pose.pose.position.x = imu_odom_x_;
//   odom.pose.pose.position.y = imu_odom_y_;

//   odom.pose.pose.orientation.w = yis_sys.q0;
//   odom.pose.pose.orientation.z = yis_sys.q3;
//   imu_odom_pub.publish(odom);
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle imu_handle;
    imu_pub = imu_handle.advertise<sensor_msgs::Imu>("yis_imu",10);
    mag_pub = imu_handle.advertise<sensor_msgs::MagneticField>("yis_mag",10);
    // ros::Publisher imu_odom_pub = imu_handle.advertise<nav_msgs::Odometry>("yis_imu_odom", 10);
    // ros::Subscriber odom_sub = imu_handle.subscribe("odom", 10, myOdomCallback);


    std::string param_com_device_path;
    imu_handle.param("/imu/com_device_path", param_com_device_path, std::string("/dev/ros/imu"));

    char com_device_path[50];
    strcpy(com_device_path, param_com_device_path.c_str());

    memcpy(yis_sys.dev,com_device_path,sizeof(com_device_path));

    yis_sys.com_device = open_com_device(yis_sys.dev);
    set_speed(yis_sys.com_device,460800);
    set_parity(yis_sys.com_device,8,1,'N');
    ROS_ERROR("com_device: %d", yis_sys.com_device);

    ros::Rate loop_rate(110);
    while(ros::ok())
    {
        handle_receive_data(yis_sys);
        // pub_imu_odom(imu_odom_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return (0);
}
