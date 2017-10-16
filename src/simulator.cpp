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
#include "simulator.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <limits>
#include <string>

namespace enc = sensor_msgs::image_encodings;

Simulator::Simulator(ros::NodeHandle nh, TWorldInfoBuffer &buffer):
  buffer_(buffer), nh_(nh)
{
  path_ = nh_.advertise<geometry_msgs::PoseArray>("/path", 1);

//  odom_ = nh_.subscribe("odom", 1000, &Simulator::odomCallBack, this);

//  image_transport::ImageTransport it(nh);
//  ogmap_ = it.subscribe("map_image/full", 1, &Simulator::ogMapCallBack, this);
}
