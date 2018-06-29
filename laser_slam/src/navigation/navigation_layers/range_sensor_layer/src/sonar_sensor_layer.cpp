#include <range_sensor_layer/range_sensor_layer.h>
#include <boost/algorithm/string.hpp>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include "range_sensor_layer/sonar_msgs.h"


PLUGINLIB_EXPORT_CLASS(range_sensor_layer::SonarSensorLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;

namespace range_sensor_layer
{

SonarSensorLayer::SonarSensorLayer() {}

void SonarSensorLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  buffered_readings_ = 0;
  last_reading_time_ = ros::Time::now();
  default_value_ = to_cost(0.5);

  matchSize();
  min_x_ = min_y_ = -std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::max();

  // Default topic names list contains a single topic: /sonar
  // We use the XmlRpcValue constructor that takes a XML string and reading start offset
  const char* xml = "<value><array><data><value>/sonar</value></data></array></value>";
  int zero_offset = 0;
  std::string topics_ns;
  XmlRpc::XmlRpcValue topic_names(xml, &zero_offset);

  nh.param("ns", topics_ns, std::string());
  nh.param("topics", topic_names, topic_names);
  nh.param("error_data_limit", error_data_limit_, 2.5);    //default sensor is wrong if data value is over 2.5m.
  nh.param("keep_time", keep_time_, 5.0);                  //default sensor data keep 5.0s valid time.

  InputSensorType input_sensor_type = ALL;
  std::string sensor_type_name;
  nh.param("input_sensor_type", sensor_type_name, std::string("ALL"));

  boost::to_upper(sensor_type_name);
  ROS_INFO("%s: %s as input_sensor_type given", name_.c_str(), sensor_type_name.c_str());

  sensor_type_name = "VARIABLE";
  if (sensor_type_name == "VARIABLE")
    input_sensor_type = VARIABLE;
  else if (sensor_type_name == "FIXED")
    input_sensor_type = FIXED;
  else if (sensor_type_name == "ALL")
    input_sensor_type = ALL;
  else
  {
    ROS_ERROR("%s: Invalid input sensor type: %s", name_.c_str(), sensor_type_name.c_str());
  }

  // Validate topic names list: it must be a (normally non-empty) list of strings
  if ((topic_names.valid() == false) || (topic_names.getType() != XmlRpc::XmlRpcValue::TypeArray))
  {
    ROS_ERROR("Invalid topic names list: it must be a non-empty list of strings");
    return;
  }

  if (topic_names.size() < 1)
  {
    // This could be an error, but I keep it as it can be useful for debug
    ROS_WARN("Empty topic names list: range sensor layer will have no effect on costmap");
  }


/*
 *   laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
  laser_scan_filter_ =
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                        *tf_,
                                                        odom_frame_id_,
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                   this, _1));
*/
  global_frame_ = layered_costmap_->getGlobalFrameID();
  //ROS_ERROR("sonar layer initial 1");
  /*sonar_sub_ = new message_filters::Subscriber<range_sensor_layer::sonar_msgs>(nh, "/sonar_msg_all", 10);
  sonar_filter_ = new tf::MessageFilter<range_sensor_layer::sonar_msgs>(*sonar_sub_,
                                                          *tf_,
                                                          global_frame_,
                                                          10);
  sonar_filter_->registerCallback(boost::bind(&SonarSensorLayer::bufferIncomingRangeMsg,this, _1));*/

  //gavin  sonar_sub_ = nh.subscribe("/sonar_msgs",5,&SonarSensorLayer::bufferIncomingRangeMsg, this);
  // Traverse the topic names list subscribing to all of them with the same callback method
  for (unsigned int i = 0; i < topic_names.size(); i++)
  {
    if (topic_names[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_WARN("Invalid topic names list: element %d is not a string, so it will be ignored", i);
    }
    else
    {
      std::string topic_name(topics_ns);
      if ((topic_name.size() > 0) && (topic_name.at(topic_name.size() - 1) != '/'))
        topic_name += "/";
      topic_name += static_cast<std::string>(topic_names[i]);

      if (input_sensor_type == VARIABLE)
        processRangeMessageFunc_ = boost::bind(&SonarSensorLayer::processVariableRangeMsg, this, _1);
      else if (input_sensor_type == FIXED)
        processRangeMessageFunc_ = boost::bind(&SonarSensorLayer::processFixedRangeMsg, this, _1);
      else if (input_sensor_type == ALL)
        processRangeMessageFunc_ = boost::bind(&SonarSensorLayer::processRangeMsg, this, _1);
      else
      {
        ROS_ERROR(
            "%s: Invalid input sensor type: %s. Did you make a new type and forgot to choose the subscriber for it?",
            name_.c_str(), sensor_type_name.c_str());
      }

      ROS_INFO("topic name:%s",topic_name.c_str());
      boost::shared_ptr < message_filters::Subscriber<range_sensor_layer::sonar_msgs>
      > sub(new message_filters::Subscriber<range_sensor_layer::sonar_msgs>(nh, topic_name, 50));

      boost::shared_ptr < tf::MessageFilter<range_sensor_layer::sonar_msgs>
      > filter(new tf::MessageFilter<range_sensor_layer::sonar_msgs>(*sub, *tf_, global_frame_, 50));

      filter->registerCallback(boost::bind(&SonarSensorLayer::bufferIncomingRangeMsg, this, _1));


      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);

      //observation_notifiers_.back()->setTolerance(ros::Duration(0.05));

      //range_subs_.push_back(nh.subscribe(topic_name, 10, &SonarSensorLayer::bufferIncomingRangeMsg, this));

     // ROS_INFO("SonarSensorLayer: subscribed to topic %s", range_subs_.back().getTopic().c_str());
    }
  }
  //ROS_ERROR("sonar layer initial 2");
  dsrv_ = new dynamic_reconfigure::Server<range_sensor_layer::RangeSensorLayerConfig>(nh);
  dynamic_reconfigure::Server<range_sensor_layer::RangeSensorLayerConfig>::CallbackType cb = boost::bind(
      &SonarSensorLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  //global_frame_ = layered_costmap_->getGlobalFrameID();
}


double SonarSensorLayer::gamma(double theta)
{
    if(fabs(theta)>max_angle_)
        return 0.0;
    else
        return 1;// - pow(theta/max_angle_, 2);
}

double SonarSensorLayer::delta(double phi)
{
    return 1 - (1+tanh(2*(phi-phi_v_)))/2;
}


double SonarSensorLayer::sensor_model(double r, double phi, double theta)
{
    double lbda = gamma(theta);//delta(phi)*gamma(theta);

    double delta = resolution_;

    if(phi >= 0.0 and phi < r - delta)
    {
        return (1- lbda) * (0.5);
    }
    else if(phi < r + delta)
    {
        return lbda * 0.5 + 0.5;
    }
    else
    {
        return 0.5;
    }
}


void SonarSensorLayer::reconfigureCB(range_sensor_layer::RangeSensorLayerConfig &config, uint32_t level)
{
  phi_v_ = config.phi;
  max_angle_ = config.max_angle;
  no_readings_timeout_ = config.no_readings_timeout;
  clear_threshold_ = config.clear_threshold;
  mark_threshold_ = config.mark_threshold;
  clear_on_max_reading_ = config.clear_on_max_reading;
    
  if(enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
  }
}

//void SonarSensorLayer::bufferIncomingRangeMsg(const sensor_msgs::RangeConstPtr& range_message)
void SonarSensorLayer::bufferIncomingRangeMsg(const range_sensor_layer::sonar_msgsConstPtr& range_message)
{
  //range_sensor_layer::sonar_msgs msg = *range_message;
  boost::mutex::scoped_lock lock(range_message_mutex_);
  range_msgs_buffer_.push_back(*range_message);
  //ROS_ERROR("received sonar last range:%f,time:%f,now:%f",(msg.sonars.end()-6)->range,msg.header.stamp.toSec(),  ros::Time::now().toSec());
}

void SonarSensorLayer::updateCostmap()
{
  std::list<range_sensor_layer::sonar_msgs> range_msgs_buffer_copy;

  range_message_mutex_.lock();
  range_msgs_buffer_copy = std::list<range_sensor_layer::sonar_msgs>(range_msgs_buffer_);
  range_msgs_buffer_.clear();
  range_message_mutex_.unlock();


  if((0 == range_msgs_buffer_copy.size()) && (0 == time_range_msgs_buffer_.size()))
  {
      return;
  }

  for(std::list<range_sensor_layer::sonar_msgs>::iterator it = range_msgs_buffer_copy.begin();
          it != range_msgs_buffer_copy.end(); it++)
  {
      time_range_msgs_buffer_.push_back(*it);
  }
  clearCostmap();
  range_sensor_layer::sonar_msgs latest = *(time_range_msgs_buffer_.begin());
  //std::list<range_sensor_layer::sonar_msgs>::iterator latest = time_range_msgs_buffer_.begin();
  //ROS_INFO("time range msgs buffer size:%d,keep_time_:%f",time_range_msgs_buffer_.size(),keep_time_);
  if(time_range_msgs_buffer_.size() >= 2)
  {
      for(std::list<range_sensor_layer::sonar_msgs>::iterator it = time_range_msgs_buffer_.begin();
            it != time_range_msgs_buffer_.end(); it++)
        {
            if(latest.header.stamp < it->header.stamp)
            {
                latest = *it;
            }
        }
  }
  time_range_msgs_buffer_.clear();
  time_range_msgs_buffer_.push_back(latest);
  if(ros::Time::now() - latest.header.stamp < ros::Duration(keep_time_))
  {
      processVariableRangeMsg(latest); //processRangeMessageFunc_(latest);
  }
  else
  {
      ROS_ERROR("sensor data invalid because of time over:%f,range:%f",latest.header.stamp.toSec(),(latest.sonars.end()-1)->range);
  }
  //processRangeMessageFunc_(latest);

  //ROS_ERROR("sonar all msg have buffer size:%d",time_range_msgs_buffer_.size());
}

void SonarSensorLayer::processRangeMsg(range_sensor_layer::sonar_msgs& range_message)
{
  /*if (range_message.min_range == range_message.max_range)
    processFixedRangeMsg(range_message);
  else
    processVariableRangeMsg(range_message);*/
}

void SonarSensorLayer::processFixedRangeMsg(range_sensor_layer::sonar_msgs& range_message)
{
  /*if (!isinf(range_message.range))
  {
    ROS_ERROR_THROTTLE(1.0,
        "Fixed distance ranger (min_range == max_range) in frame %s sent invalid value. Only -Inf (== object detected) and Inf (== no object detected) are valid.",
        range_message.header.frame_id.c_str());
    return;
  }

  bool clear_sensor_cone = false;

  if (range_message.range > 0) //+inf
  {
    if (!clear_on_max_reading_)
      return; //no clearing at all

    clear_sensor_cone = true;
  }

  range_message.range = range_message.min_range;

  updateCostmap(range_message, clear_sensor_cone);*/
}

void SonarSensorLayer::processVariableRangeMsg(range_sensor_layer::sonar_msgs& range_message)
{
    //handle all sonar sensor data.
    for(std::vector<sensor_msgs::Range>::iterator it=range_message.sonars.begin();it != range_message.sonars.end();it++)
    {
        if(it->range > error_data_limit_)
        {
            //if data is over limit,we think data is invalid.
            continue;
        }
        if (it->range > it->max_range)
        {
            it->range = it->max_range;
        }
        bool clear_sensor_cone = false;

        if (it->range == it->max_range && clear_on_max_reading_)
          clear_sensor_cone = true;

        updateCostmap(*it, clear_sensor_cone);
        buffered_readings_++;
    }
    last_reading_time_ = ros::Time::now();
}

void SonarSensorLayer::updateCostmap(sensor_msgs::Range& range_message, bool clear_sensor_cone)
{
  max_angle_ = range_message.field_of_view/2;

  geometry_msgs::PointStamped in, out;
  in.header.stamp = range_message.header.stamp;
  in.header.frame_id = range_message.header.frame_id;
  double ox,oy;
  try
  {

  if(!tf_->waitForTransform(global_frame_, in.header.frame_id,
        in.header.stamp, ros::Duration(0.1)) )
  {
      ROS_ERROR_THROTTLE(1.0, "gavin Sonar sensor layer can't transform from %s to %s at %f",
        global_frame_.c_str(), in.header.frame_id.c_str(),
        in.header.stamp.toSec());
      return;
  }
  //ROS_INFO("process sonar all %s,range:%f", range_message.header.frame_id.c_str(),range_message.range);
  tf_->transformPoint (global_frame_, in, out);

  ox = out.point.x, oy = out.point.y;

  in.point.x = range_message.range;

  tf_->transformPoint(global_frame_, in, out);
  }
   catch (tf::TransformException& ex)
  {
  	 ROS_ERROR("TF Exception that should never happen for frame: %s, frame: %s, %s", global_frame_.c_str(),
                          in.header.frame_id.c_str(), ex.what());
         return;
  }
  double tx = out.point.x, ty = out.point.y;

  // calculate target props
  double dx = tx-ox, dy = ty-oy,
        theta = atan2(dy,dx), d = sqrt(dx*dx+dy*dy);

  // Integer Bounds of Update
  int bx0, by0, bx1, by1;

  // Bounds includes the origin
  worldToMapNoBounds(ox, oy, bx0, by0);
  bx1 = bx0;
  by1 = by0;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update Map with Target Point
  unsigned int aa, ab;
  if(worldToMap(tx, ty, aa, ab))
  {
    if(clear_sensor_cone)
    {
        setCost(aa, ab, 10);
    }
    else
    {
        setCost(aa, ab, 233);
    }
    touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
  }

  double mx, my;
  int a, b;

  if(d  < 0.5)
  {
     d = 0.5;
  }
  // Update left side of sonar cone
  mx = ox + cos(theta-max_angle_) * d * 1.2;
  my = oy + sin(theta-max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, a, b);
  bx0 = std::min(bx0, a);
  bx1 = std::max(bx1, a);
  by0 = std::min(by0, b);
  by1 = std::max(by1, b);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update right side of sonar cone
  mx = ox + cos(theta+max_angle_) * d * 1.2;
  my = oy + sin(theta+max_angle_) * d * 1.2;

  worldToMapNoBounds(mx, my, a, b);
  bx0 = std::min(bx0, a);
  bx1 = std::max(bx1, a);
  by0 = std::min(by0, b);
  by1 = std::max(by1, b);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Limit Bounds to Grid
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min((int)size_x_, bx1);
  by1 = std::min((int)size_y_, by1);

  //ROS_ERROR("bx,by:%d,%d,%d,%d,range:%f",bx0,bx1,by0,by1,range_message.range);
  for(unsigned int x=bx0; x<=(unsigned int)bx1; x++)
  {
    for(unsigned int y=by0; y<=(unsigned int)by1; y++)
    {
      double wx, wy;
      mapToWorld(x,y,wx,wy);
      update_cell(ox, oy, theta, range_message.range, wx, wy, clear_sensor_cone);
    }
  }
}

void SonarSensorLayer::update_cell(double ox, double oy, double ot, double r, double nx, double ny, bool clear)
{
  unsigned int x, y;
  if(worldToMap(nx, ny, x, y))
  {
    double dx = nx-ox, dy = ny-oy;
    double theta = atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);
    double phi = sqrt(dx*dx+dy*dy);
    double sensor = 0.0;
    if(!clear)
    {
        sensor = sensor_model(r,phi,theta);
    }
    else
    {
        if(fabs(theta) > max_angle_)
        {
            sensor = 0.5;
        }
        else
        {
            if(phi >= 0.0 and phi <= r+0.05 )
                sensor = 0.0;
            else
                sensor = 0.5;
        }
    }
    double prior = to_prob(getCost(x,y));
    double prob_occ = sensor * prior;
    double prob_not = (1 - sensor) * (1 - prior);
    double new_prob = prob_occ/(prob_occ+prob_not);

    //ROS_INFO("%f %f | %f %f = %f", dx, dy, theta, phi, sensor);
    //ROS_INFO("%f | %f %f | %f", prior, prob_occ, prob_not, new_prob);
      unsigned char c = to_cost(new_prob);
      setCost(x,y,c);
  }
}

void SonarSensorLayer::clearCostmap()
{
    for(unsigned int x=0; x<size_x_; x++)
    {
      for(unsigned int y=0; y<size_y_; y++)
      {
        setCost(x,y,default_value_);
      }
    }
}

void SonarSensorLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (layered_costmap_->isRolling())
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  updateCostmap();

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::min();

  if (!enabled_)
  {
    current_ = true;
    return;
  }
  
  if (buffered_readings_ == 0)
  {
    if (no_readings_timeout_ > 0.0 &&
        (ros::Time::now() - last_reading_time_).toSec() > no_readings_timeout_)
    {
      ROS_WARN_THROTTLE(2.0, "No range readings received for %.2f seconds, " \
                             "while expected at least every %.2f seconds.",
               (ros::Time::now() - last_reading_time_).toSec(), no_readings_timeout_);
      current_ = false;
    }
  }

}

void SonarSensorLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = to_cost(clear_threshold_), mark = to_cost(mark_threshold_);

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      unsigned char prob = costmap_[it];
      unsigned char current;
      if(prob==costmap_2d::NO_INFORMATION)
      {
        it++;
        continue;
      }
      else if(prob>mark)
        current = costmap_2d::LETHAL_OBSTACLE;
      else if(prob<clear)
        current = costmap_2d::FREE_SPACE;
      else
      {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];

      if (old_cost == NO_INFORMATION || old_cost < current)
        master_array[it] = current;
      it++;
    }
  }

  buffered_readings_ = 0;
  current_ = true;
}

void SonarSensorLayer::reset()
{
  ROS_DEBUG("Reseting range sensor layer...");
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

void SonarSensorLayer::deactivate()
{
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
        if (observation_subscribers_[i] != NULL)
            observation_subscribers_[i]->unsubscribe();
    }
  range_msgs_buffer_.clear();
}

void SonarSensorLayer::activate()
{
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
        if (observation_subscribers_[i] != NULL)
            observation_subscribers_[i]->subscribe();
    }
    range_msgs_buffer_.clear();
}

} // end namespace
