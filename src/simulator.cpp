/*! @file
 *
 *  @brief TODO
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#include "simulator.h"
#include "globalmap.h"

#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "prm_sim/RequestGoal.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <limits>
#include <string>

namespace enc = sensor_msgs::image_encodings;

Simulator::Simulator(ros::NodeHandle nh, TWorldInfoBuffer &buffer):
  buffer_(buffer), nh_(nh), it_(nh)
{
  path_     = nh_.advertise<geometry_msgs::PoseArray>("path", 1);
  overlay_  = it_.advertise("prm", 1);

  reqGoal_ = nh_.advertiseService("request_goal", &Simulator::requestGoal, this);

  //Get parameters from command line
  ros::NodeHandle pn("~");
  double mapSize;
  double mapResolution;
  double robotDiameter;
  pn.param<double>("map_size", mapSize, 20.0);
  pn.param<double>("resolution", mapResolution, 0.1);
  pn.param<double>("robot_diameter", robotDiameter, 0.2);

  //Update default potentially... TODO
  gmap_.setMapSize(mapSize);
  gmap_.setResolution(mapResolution);
  gmap_.setRobotDiameter(robotDiameter);
}


bool Simulator::requestGoal(prm_sim::RequestGoal::Request &req, prm_sim::RequestGoal::Response &res)
{
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.x, (long int)req.y);
  res.ack = true;
  ROS_INFO("sending back response: [%d]", res.ack);
  return true;
}
