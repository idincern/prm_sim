/*! @file
 *
 *  @brief Simulation for robot path finding.
 *
 *  Using an internal LD-PRM path planner, this class listens
 *  for goal requests on /request_goal then builds a PRM network
 *  within a supplied configuration space.
 *  The PRM network is sent as an image to /prm, and the path waypoints
 *  between robot and goal are sent as a PoseArray to /path.
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#include "simulator.h"

#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "prm_sim/RequestGoal.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace enc = sensor_msgs::image_encodings;

static const double DEF_ROBOT_DIAMETER = 0.2; /*!< Default robot diameter is 0.2m */
static const int MAX_BUILD_ROUNDS = 5;        /*!< The max amount of times the builder is allowed to plan a path towards a goal */

Simulator::Simulator(ros::NodeHandle nh, TWorldDataBuffer &buffer):
  buffer_(buffer), nh_(nh), it_(nh)
{
  pathPub_      = nh_.advertise<geometry_msgs::PoseArray>("path", 1);
  overlayPub_   = it_.advertise("prm", 1);
  reqGoal_      = nh_.advertiseService("request_goal", &Simulator::requestGoal, this);

  //Get parameters from command line
  ros::NodeHandle pn("~");
  double mapSize;
  double mapResolution;
  int density;

  pn.param<double>("map_size", mapSize, PLANNER_DEF_MAP_SIZE);
  pn.param<double>("resolution", mapResolution, PLANNER_DEF_MAP_RES);
  pn.param<int>("density", density, PLANNER_DEF_DENSITY);
  pn.param<double>("robot_diameter", robotDiameter_, DEF_ROBOT_DIAMETER);

  ROS_INFO("Init with: map_size={%.1f} resolution={%.1f} robot_diameter={%.1f} density={%d}",
           mapSize, mapResolution, robotDiameter_, density);

  planner_ = PrmPlanner(mapSize, mapResolution, density);
}

void Simulator::overlayThread(){
  cv::Mat msg;

  while(ros::ok()){
    if(overlayContainer_.dirty){
      //We make a copy of the prmOverlay,
      //otherwise we retain a reference to it which can change
      //We only want to see change when dirty is set to true
      overlayContainer_.access.lock();

      overlayContainer_.data.copyTo(msg);
      overlayContainer_.dirty = false;

      overlayContainer_.access.unlock();
      ROS_INFO("Updating PRM overlay...");
    }

    if(!msg.empty())
      sendOverlay(msg);
  }
}

void Simulator::plannerThread() {
  //Wait until some data has arrived in the world information buffer
  ROS_INFO("Waiting to receive world data...");
  waitForWorldData();
  ROS_INFO("Ready to recieve requests...");

  while(ros::ok()){
    //Only plan if a new goal has been recieved
    if(goalContainer_.dirty)
    {
      //Get the new goal
      goalContainer_.access.lock();
      TGlobalOrd currentGoal = goalContainer_.data;
      goalContainer_.dirty = false;
      goalContainer_.access.unlock();

      //Recieve new information from the world buffer
      consumeWorldData(cspace_, robotPos_);

      //Update the reference for the localMap
      TGlobalOrd robotOrd = {robotPos_.position.x, robotPos_.position.y};
      ROS_INFO("Setting reference: {%.1f, %.1f}", robotOrd.x, robotOrd.y);
      planner_.setReference(robotOrd);

      if(cspace_.empty()){
        //Something has gone wrong during image transmission,
        //skip execution for this goal and hope new data has arived
        //on the next go around
        ROS_ERROR("Empty OgMap");
        continue;
      }

      //Copy to the prm overlay before expanding config space
      overlayContainer_.access.lock();
      cv::cvtColor(cspace_, overlayContainer_.data, CV_GRAY2BGR);
      overlayContainer_.access.unlock();

      //Expand the configuration space
      planner_.expandConfigSpace(cspace_, robotDiameter_);

      //Validate both ordinates
      if(!planner_.ordinateAccessible(cspace_, robotOrd)){
        ROS_ERROR("Robot ordinates {%.1f, %.1f} are not accessible",
                  robotOrd.x, robotOrd.y);
        continue;
      }

      if(!planner_.ordinateAccessible(cspace_, currentGoal)){
        ROS_ERROR("Goal ordinates {%.1f, %.1f} are not accessible",
                  currentGoal.x, currentGoal.y);
        continue;
      }

      //Start the build process
      ROS_INFO("Starting build: {%.1f, %.1f} to {%.1f, %.1f}",
               robotOrd.x, robotOrd.y, currentGoal.x, currentGoal.y);

      std::vector<TGlobalOrd> path;
      int round(0);
      //While we haven't found a path and the rounds a less than the max and ros is okay,
      //build more nodes and try to find a path
      while(path.size() == 0 && round < MAX_BUILD_ROUNDS && ros::ok()){
        ROS_INFO("  Building nodes...");
        path = planner_.build(cspace_, robotOrd, currentGoal);

        //Update PRM overlay with network and potentially path
        overlayContainer_.access.lock();

        planner_.showOverlay(overlayContainer_.data, path);
        overlayContainer_.dirty = true;

        overlayContainer_.access.unlock();
        round++;
      }

      if(path.size() > 0){
        //Send path information
        sendPath(path);
      } else {
        ROS_WARN("  Could not find path. Perhaps choose a closer goal?");
      }
    }
  }
}

void Simulator::waitForWorldData(){
  //We must wait until information about the world has been recieved
  //so that we can begin building the prm
  while(ros::ok()){
    int mapSz, poseSz;
    buffer_.access.lock();
    mapSz = buffer_.ogMapDeq.size();
    poseSz = buffer_.poseDeq.size();
    buffer_.access.unlock();

    if(mapSz > 0 && poseSz > 0){
      break;
    }
  }
}

bool Simulator::requestGoal(prm_sim::RequestGoal::Request &req, prm_sim::RequestGoal::Response &res)
{
  ROS_INFO("Goal request: x=%.1f, y=%.1f", (double)req.x, (double)req.y);
  goalContainer_.access.lock();

  goalContainer_.data.x = req.x;
  goalContainer_.data.y = req.y;
  goalContainer_.dirty = true;

  goalContainer_.access.unlock();

  res.ack = true;

  ROS_INFO("Sending back goal response: [%d]", res.ack);
  return true;
}

void Simulator::consumeWorldData(cv::Mat &ogMap, geometry_msgs::Pose &robotPos){
  //Get information about the world if available
  buffer_.access.lock();
  if(buffer_.ogMapDeq.size() > 0){
    ogMap = buffer_.ogMapDeq.front();
    buffer_.ogMapDeq.pop_front();
  }

  if(buffer_.poseDeq.size() > 0){
    robotPos = buffer_.poseDeq.back();
    buffer_.poseDeq.pop_front();
  }
  buffer_.access.unlock();
}

void Simulator::sendOverlay(cv::Mat &overlay){
  //Show the overlay of the prm
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", overlay).toImageMsg();
  overlayPub_.publish(msg);
}

void Simulator::sendPath(std::vector<TGlobalOrd> path){
  if(path.size() > 0){
    //Send the waypoints
    geometry_msgs::PoseArray posePath;
    for(auto const &waypoint: path) {
      geometry_msgs::Pose w;
      w.position.x = waypoint.x;
      w.position.y = waypoint.y;
      w.position.z = robotPos_.position.z; //Just send the z value of the original robot position

      posePath.poses.push_back(w);
    }

    pathPub_.publish(posePath);
    ROS_INFO("Sent path information...");
  }
}
