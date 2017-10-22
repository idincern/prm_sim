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

static std::mutex              GoalAccess;  /*!< A mutex that locks access/waits for a goal to be set */
static std::condition_variable NewGoal;     /*!< Used to wait on a goal set by user */

Simulator::Simulator(ros::NodeHandle nh, TWorldDataBuffer &buffer):
  buffer_(buffer), nh_(nh), it_(nh)
{
  pathPub_     = nh_.advertise<geometry_msgs::PoseArray>("path", 1);
  overlayPub_  = it_.advertise("prm", 1);
  reqGoal_  = nh_.advertiseService("request_goal", &Simulator::requestGoal, this);

  //Get parameters from command line
  ros::NodeHandle pn("~");
  double mapSize;
  double mapResolution;
  pn.param<double>("map_size", mapSize, 20.0);
  pn.param<double>("resolution", mapResolution, 0.1);
  pn.param<double>("robot_diameter", robotDiameter_, 0.2);

  ROS_INFO("Init with: map_size={%.1f} resolution={%.1f} robot_diameter={%.1f}",
           mapSize, mapResolution, robotDiameter_);

  planner_ = PrmPlanner(mapSize, mapResolution);
}

void Simulator::overlayThread(){
  cv::Mat msg;

  while(ros::ok()){
    if(overlayContainer_.dirty){
      //We make a copy of the prmOverlay,
      //otherwise we retain a reference to it which can change
      //We only want to see change when dirty is set to true
      overlayContainer_.access.lock();
      overlayContainer_.prmOverlay.copyTo(msg);
      overlayContainer_.access.unlock();

      overlayContainer_.dirty = false;
      ROS_INFO("Updating PRM overlay...");
    }

    if(!msg.empty())
      sendOverlay(msg);
  }
}


void Simulator::plannerThread() {
  //Wait until some data has arrived in the world information buffer
  waitForWorldData();

  ROS_INFO("Ready to recieve requests...");
  std::unique_lock<std::mutex> lock(GoalAccess);

  while(ros::ok()){
    //Wait until a new goal has been recieved
    NewGoal.wait(lock);
    TGlobalOrd currentGoal = goal_; //TODO: Atomic copy?

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

    //Create colour copy of the OgMap
    cv::Mat colourMap;
    overlayContainer_.access.lock();
    cv::cvtColor(cspace_, overlayContainer_.prmOverlay, CV_GRAY2BGR);
    overlayContainer_.access.unlock();

    //Expand the configuration space
    //TODO: Examine what cspace is doing...
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

    int cnt = 0;
    std::vector<TGlobalOrd> path;
    while(path.size() == 0){
      ROS_INFO("  Building nodes...");
      path = planner_.build(cspace_, robotOrd, currentGoal);

      //Update PRM overlay with network and potentially path
      overlayContainer_.access.lock();
      planner_.showOverlay(overlayContainer_.prmOverlay, path);
      overlayContainer_.access.unlock();
      overlayContainer_.dirty = true;

      if(cnt++ > 6){ //TODO: make number come in on command line...
        ROS_INFO("  Could not find path. Perhaps choose a closer goal?");
        break;
      }
    }

    //Send path information
    sendPath(path);
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

  goal_.x = req.x;
  goal_.y = req.y;

  NewGoal.notify_one();

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
      w.position.z = robotPos_.position.z;

      posePath.poses.push_back(w);
    }

    pathPub_.publish(posePath);
    ROS_INFO("Sent path information...");
  }
}
