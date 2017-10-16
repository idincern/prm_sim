/*! @file
 *
 *  @brief An interface for accessing information about the robot's world.
 *
 *  Using roscore to connect to other ros nodes, this object
 *  obtains information from /odom (robot's pose), and /map_image/full (OgMap).
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#include "worldretrieve.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <limits>
#include <string>

namespace enc = sensor_msgs::image_encodings;

WorldRetrieve::WorldRetrieve(ros::NodeHandle nh, TWorldInfoBuffer &buffer):
  buffer_(buffer), nh_(nh)
{
  odom_ = nh_.subscribe("odom", 1000, &WorldRetrieve::odomCallBack, this);

  image_transport::ImageTransport it(nh);
  ogmap_ = it.subscribe("map_image/full", 1, &WorldRetrieve::ogMapCallBack, this);
}

void WorldRetrieve::heartbeatThread(void)
{
  while(ros::ok()){
    ROS_INFO("I am the watcher of worlds..."); // TODO: Better message/is this necessary?

    //Display a heartbeat message every so often
    std::this_thread::sleep_for (std::chrono::seconds(5));
  }
}

//TODO: Should we sepearte the world buffer... both callbacks locking the buffer
void WorldRetrieve::odomCallBack(const nav_msgs::OdometryConstPtr &msg){
  geometry_msgs::Pose pose = msg->pose.pose;

  buffer_.access.lock();
  buffer_.poseDeq.push_back(pose);
  buffer_.access.unlock();
}

void WorldRetrieve::ogMapCallBack(const sensor_msgs::ImageConstPtr &msg){
  cv_bridge::CvImagePtr cvPtr;

  //TODO: Enforce grey images on queue
  try
  {
    if (enc::isColor(msg->encoding))
      cvPtr = cv_bridge::toCvCopy(msg, enc::BGR8);
    else
      cvPtr = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  buffer_.access.lock();

  buffer_.ogMapDeq.push_back(cvPtr->image);

  if(buffer_.ogMapDeq.size() > 2){
    buffer_.ogMapDeq.pop_front();
  }

  buffer_.access.unlock();
}