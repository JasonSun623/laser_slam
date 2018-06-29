
/*
 Software License Agreement (BSD License)

 Copyright (c) 2012, Scott Niekum
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the Willow Garage nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 author: Scott Niekum
*/


#include <std_msgs/Bool.h>
#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
//#include <ar_track_alvar_msgs/Id.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarId.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <ar_track_alvar/ParamsConfig.h>
#include <ar_track_alvar/Pose.h>


using namespace alvar;
using namespace std;

bool init=true;
Camera *cam;
cv_bridge::CvImagePtr cv_ptr_;
image_transport::Subscriber cam_sub_;
ros::Publisher arMarkerPub_;
ros::Publisher arNavigationPub_;
ros::Publisher rvizMarkerPub_;
ar_track_alvar_msgs::AlvarMarkers arPoseMarkers_;
ar_track_alvar_msgs::AlvarId arId_;
visualization_msgs::Marker rvizMarker_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;

bool enableSwitched = false;
bool enabled = true;
double debug;
double max_x = -1000;
double min_x = 1000;
double max_y = -1000; 
double min_y = 1000;
double max_frequency;
double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic; 
std::string cam_info_topic; 
std::string output_frame;

void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg);


void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg)
{
    //If we've already gotten the cam info, then go ahead
	if(cam->getCamInfo_){
		try{
			tf::StampedTransform CamToOutput;
    			try{
					tf_listener->waitForTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, ros::Duration(1.0));
					tf_listener->lookupTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, CamToOutput);
   				}
    			catch (tf::TransformException ex){
      				ROS_ERROR("%s",ex.what());
    			}

            ros::Time s_ros, e_ros;
            s_ros = ros::Time::now();

            //Convert the image
            cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            cv::subtract(cv::Scalar::all(255),cv_ptr_->image,cv_ptr_->image);
            //Get the estimated pose of the main markers by using all the markers in each bundle

            // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
            // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
            // do this conversion here -jbinney
            IplImage ipl_image = cv_ptr_->image;
            
            marker_detector.Detect(&ipl_image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true);
            arPoseMarkers_.markers.clear ();
            double minimal_distance = 10000000;
            int closest_camera_index = -1;
            float closest_x  = -1;
            float closest_y  = -1;
            float closest_z  = -1;
            float closest_x2  = -1;
            float closest_y2  = -1;
            float closest_z2  = -1;
            int   closest_id = -1;
            float closest_th = -1;
            float closest_roll = -1;
            float closest_pitch = -1;
            

            
			for (size_t i=0; i<marker_detector.markers->size(); i++) 
			{
				//Get the pose relative to the camera
        		int id = (*(marker_detector.markers))[i].GetId(); 
                                alvar::Pose p = (*(marker_detector.markers))[i].pose;
				double px = p.translation[0]/100.0;
				double py = p.translation[1]/100.0;
				double pz = p.translation[2]/100.0;
				double qx = p.quaternion[1];
				double qy = p.quaternion[2];
				double qz = p.quaternion[3];
				double qw = p.quaternion[0];

                //ROS_INFO_STREAM("\nMarker ID=" << arId_.id << "\npx=" << px << "\npy=" << py << "\npz=" << pz );

                tf::Quaternion rotation (qx,qy,qz,qw);
                tf::Vector3 origin (px,py,pz);       

                tf::Transform t (rotation, origin);
                tf::Vector3 markerOrigin (0, 0, 0);

                //tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
                //tf::Transform markerPose = t * m; // marker pose in the camera frame
                tf::Transform markerPose = t;

                //tf::StampedTransform markerPose2;
                //tf::Transform markerPose = t * rotate_to_ros.t(); // marker pose in the camera frame

                tf::Vector3 z_axis_cam = tf::Transform(rotation, tf::Vector3(0,0,0)) * tf::Vector3(0, 0, 1);
//                ROS_INFO("%02i Z in cam frame: %f %f %f",id, z_axis_cam.x(), z_axis_cam.y(), z_axis_cam.z());
                /// as we can't see through markers, this one is false positive detection
                if (z_axis_cam.z() > 0)
                {
                    continue;
                } 

				//Publish the transform from the camera to the marker		
				std::string markerFrame = "ar_marker_";
				std::stringstream out;
				out << id;
				std::string id_string = out.str();
				markerFrame += id_string;

/*
                std::string cc = "cc";

				tf::StampedTransform camToMarker (t, image_msg->header.stamp, cc.c_str(), markerFrame.c_str());
    			tf_broadcaster->sendTransform(camToMarker);
				
                tf::TransformListener *listener_ = new tf::TransformListener;
                listener_->waitForTransform(cc.c_str(), markerFrame.c_str(), ros::Time(0), ros::Duration(2));

                try
                {
                  listener_->lookupTransform(cc.c_str(), markerFrame.c_str(), ros::Time(0),  markerPose2);
                }
                catch(tf::TransformException &e)
                {
                  ROS_ERROR("unable to transform");
                }
*/
				//Create the rviz visualization messages
				tf::poseTFToMsg (markerPose, rvizMarker_.pose);
				rvizMarker_.header.frame_id = image_msg->header.frame_id;
				rvizMarker_.header.stamp = image_msg->header.stamp;
				rvizMarker_.id = id;

				rvizMarker_.scale.x = 1.0 * marker_size/100.0;
				rvizMarker_.scale.y = 1.0 * marker_size/100.0;
				rvizMarker_.scale.z = 0.2 * marker_size/100.0;
				rvizMarker_.ns = "basic_shapes";
				rvizMarker_.type = visualization_msgs::Marker::CUBE;
				rvizMarker_.action = visualization_msgs::Marker::ADD;
				switch (id)
				{
				  case 0:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 1.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 1:
				    rvizMarker_.color.r = 1.0f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 1.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 2:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 1.0f;
				    rvizMarker_.color.b = 0.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 3:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 0.5f;
				    rvizMarker_.color.b = 0.5f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 4:
				    rvizMarker_.color.r = 0.5f;
				    rvizMarker_.color.g = 0.5f;
				    rvizMarker_.color.b = 0.0;
				    rvizMarker_.color.a = 1.0;
				    break;
				  default:
				    rvizMarker_.color.r = 0.5f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 0.5f;
				    rvizMarker_.color.a = 1.0;
				    break;
				}
				rvizMarker_.lifetime = ros::Duration (1.0);
				rvizMarkerPub_.publish (rvizMarker_);

				//Get the pose of the tag in the camera frame, then the output frame (usually torso)				
				tf::Transform tagPoseOutput = CamToOutput * markerPose;
 
				//Create the pose marker messages
				ar_track_alvar_msgs::AlvarMarker ar_pose_marker;
				tf::poseTFToMsg (tagPoseOutput, ar_pose_marker.pose.pose); 
                tf::Matrix3x3 ma(tagPoseOutput.getRotation());
                double roll,pitch,yaw;                                
                ma.getRPY(roll,pitch,yaw);

                yaw = -yaw;
                if (yaw < 3.1415/2)
                {
                  yaw = yaw + 3.1415/2;
                }
                else
                {
                  yaw = yaw - 3.145*3/2;
                }
               

                double a,b,c,size, a2,b2,c2;
                a = ar_pose_marker.pose.pose.position.x;
                b = ar_pose_marker.pose.pose.position.y;
                c = ar_pose_marker.pose.pose.position.z;
                size = std::sqrt((a*a)+(b*b)+(c*c));
                if (size < minimal_distance)
                {
                  minimal_distance = size;
                  closest_camera_index = i;
                  closest_x = a;
                  closest_y = b;
                  closest_z = c;
                  closest_x2 = sin(yaw)*b-cos(yaw)*a;
                  closest_y2 = -sin(yaw)*a-cos(yaw)*b;
                  closest_z2 = c;
                  closest_th = yaw;
                  closest_id = id;
                  closest_roll = roll;
                  closest_pitch = pitch;
                }

      			ar_pose_marker.header.frame_id = output_frame;
			    ar_pose_marker.header.stamp = image_msg->header.stamp;
			    ar_pose_marker.id = id;
			    arPoseMarkers_.markers.push_back (ar_pose_marker);	
			}
			arMarkerPub_.publish (arPoseMarkers_);
			arId_.header.stamp = ros::Time::now();
            arId_.id = closest_id;
            arId_.th = closest_th;
            //arId_.th = closest_th;
            arId_.x = closest_y2;
            arId_.y = -closest_x2;
            arId_.z = closest_z;
            arId_.pitch = closest_pitch;
            arId_.roll = closest_roll;
            e_ros = ros::Time::now();

            //ROS_INFO("x:%.3f,y:%.3f,th:%.3f,z:%.3f,id:%d",arId_.x,arId_.y,arId_.th,arId_.z,arId_.id);
            if (debug == 1 && !( arId_.x == -1 && arId_.y == 1 ))
            {
              if (arId_.x < min_x) min_x = arId_.x;
              if (arId_.y < min_y) min_y = arId_.y;
              if (arId_.x > max_x) max_x = arId_.x;
              if (arId_.y > max_y) max_y = arId_.y;
              ROS_INFO_STREAM("\nRx =" << (max_x - min_x)/2 << "\nRy =" << (max_y - min_y)/2 );
            }

            //ROS_INFO_STREAM("\nMarker ID=" << arId_.id << "\nx=" << arId_.x << "\ny=" << arId_.y << "\nz=" << arId_.z << "\n\nx2=" << closest_x2 << "\ny2=" << closest_y2 <<  "\nz2=" << closest_z2 << "\n\nyaw=" << arId_.th << "\nTime=" << (e_ros-s_ros).toNSec()*1e-6 );
            arNavigationPub_.publish (arId_);
                          
                          
		}
        catch (cv_bridge::Exception& e){
      		ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    	}
	}
}

void configCallback(ar_track_alvar::ParamsConfig &config, uint32_t level)
{
  ROS_INFO("AR tracker reconfigured: %s %.2f %.2f %.2f %.2f", config.enabled ? "ENABLED" : "DISABLED",
           config.max_frequency, config.marker_size, config.max_new_marker_error, config.max_track_error);

  enableSwitched = enabled != config.enabled;

  enabled = config.enabled;
  max_frequency = config.max_frequency;
  marker_size = config.marker_size;
  max_new_marker_error = config.max_new_marker_error;
  max_track_error = config.max_track_error;
}

void enableCallback(const std_msgs::BoolConstPtr& msg)
{
    enableSwitched = enabled != msg->data;
    enabled = msg->data;
}

int main(int argc, char *argv[])
{
	ros::init (argc, argv, "marker_detect");
	ros::NodeHandle n, pn("~");
	
	if(argc < 8){
		std::cout << std::endl;
		cout << "Not enough arguments provided." << endl;
		cout << "Usage: ./individualMarkersNoKinect <marker size in cm> <max new marker error> "
		     << "<max track error> <cam image topic> <cam info topic> <output frame> [ <max frequency> ]";
		std::cout << std::endl;
		return 0;
	}

	// Get params from command line
	marker_size = atof(argv[1]);
	max_new_marker_error = atof(argv[2]);
	max_track_error = atof(argv[3]);
	cam_image_topic = argv[4];
	cam_info_topic = argv[5];
    output_frame = argv[6];
    debug = atof(argv[7]);
	marker_detector.SetMarkerSize(marker_size);

  if (argc > 8)
    max_frequency = atof(argv[8]);

  // Set dynamically configurable parameters so they don't get replaced by default values
  pn.setParam("marker_size", marker_size);
  pn.setParam("max_new_marker_error", max_new_marker_error);
  pn.setParam("max_track_error", max_track_error);

  if (argc > 8)
    pn.setParam("max_frequency", max_frequency);

	cam = new Camera(n, cam_info_topic);
	tf_listener = new tf::TransformListener(n);
	tf_broadcaster = new tf::TransformBroadcaster();
	arMarkerPub_ = n.advertise < ar_track_alvar_msgs::AlvarMarkers > ("ar_pose_marker", 0);
        //arNavigationPub_ = n.advertise < ar_track_alvar_msgs::Id > ("ar_id_marker", 0);
        arNavigationPub_ = n.advertise < ar_track_alvar_msgs::AlvarId > ("star_info", 0);
	rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
	
  // Prepare dynamic reconfiguration
  dynamic_reconfigure::Server < ar_track_alvar::ParamsConfig > server;
  dynamic_reconfigure::Server<ar_track_alvar::ParamsConfig>::CallbackType f;

  f = boost::bind(&configCallback, _1, _2);
  server.setCallback(f);

	//Give tf a chance to catch up before the camera callback starts asking for transforms
  // It will also reconfigure parameters for the first time, setting the default values
	ros::Duration(1.0).sleep();
	ros::spinOnce();	
	 
	image_transport::ImageTransport it_(n);

  // Run at the configured rate, discarding pointcloud msgs if necessary
  ros::Rate rate(max_frequency);

  /// Subscriber for enable-topic so that a user can turn off the detection if it is not used without
  /// having to use the reconfigure where he has to know all parameters
  ros::Subscriber enable_sub_ = pn.subscribe("enable_detection", 1, &enableCallback);

  enableSwitched = true;
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    if (std::abs((rate.expectedCycleTime() - ros::Duration(1.0 / max_frequency)).toSec()) > 0.001)
    {
      // Change rate dynamically; if must be above 0, as 0 will provoke a segfault on next spinOnce
      ROS_DEBUG("Changing frequency from %.2f to %.2f", 1.0 / rate.expectedCycleTime().toSec(), max_frequency);
      rate = ros::Rate(max_frequency);
    }

    if (enableSwitched)
    {
      // Enable/disable switch: subscribe/unsubscribe to make use of pointcloud processing nodelet
      // lazy publishing policy; in CPU-scarce computer as TurtleBot's laptop this is a huge saving
        if (enabled)
            cam_sub_ = it_.subscribe(cam_image_topic, 1, &getCapCallback);
        else
            cam_sub_.shutdown();
        enableSwitched = false;
    }
  }

    return 0;
}
