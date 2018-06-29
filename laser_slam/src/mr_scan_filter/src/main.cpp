#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

ros::Publisher scan_filter_pub;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    double d_range = 0.0;
    double a_range = 0.0;
    sensor_msgs::LaserScan new_scan(*scan_msg);
    //ROS_INFO("ranges size:%d",new_scan.ranges.size());
    for(int i=1; i< new_scan.ranges.size()-1;i++)
    {
        if((new_scan.ranges[i] < 1.0) && (new_scan.ranges[i] > 0.01))
        {
            //ROS_INFO("ranges[%d]:%f,intensities:%f",i,new_scan.ranges[i],new_scan.intensities[i]);
            double d1 = fabs(new_scan.ranges[i] - new_scan.ranges[i-1]);
            double d2 = fabs(new_scan.ranges[i] - new_scan.ranges[i+1]);
            double d3 = fabs(new_scan.ranges[i-1] - new_scan.ranges[i+1]);
            if(d3 < 0.01)
            {
                d3 = 0.01;
            }
            a_range = (new_scan.ranges[i-1] + new_scan.ranges[i] + new_scan.ranges[i+1])/3.0;
            
            if((((d1+d2)/d3) > 2) && ((d1+d2) / a_range) > 0.4)
            {
                new_scan.ranges[i] = (new_scan.ranges[i-1] + new_scan.ranges[i+1])/2;
            }
            //ROS_INFO("(d1+d2)/d3:%f,(d1+d2) / a_range:%f",(d1+d2)/d3,(d1+d2)/a_range);
        }
    }/**/
    scan_filter_pub.publish(new_scan);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "scan_filter");
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  scan_filter_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::Subscriber scan_sub = n.subscribe("scan_origin",1000,scan_callback);


  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

