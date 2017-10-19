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

//TODO: Examine header file...
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <signal.h>

namespace enc = sensor_msgs::image_encodings;

static std::mutex              goalAccess;      /*!< TODO */
static std::mutex              pathBuilding;
static std::mutex              construction;

static std::condition_variable newGoal;      /*!< TODO */
static std::condition_variable buildGoal;
static std::condition_variable constructionComplete;

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
  pn.param<double>("map_size", mapSize, 20.0);
  pn.param<double>("resolution", mapResolution, 0.1);
  pn.param<double>("robot_diameter", robotDiameter_, 0.2);

  ROS_INFO("Init with: map_size={%.1f} resolution={%.1f} robot_diameter={%.1f}",
           mapSize, mapResolution, robotDiameter_);

  gmap_ = GlobalMap(mapSize, mapResolution);
}

//void Simulator::queryThread(){
//  //We must wait until information about the world has been recieved
//  //so that we can begin building the prm
//  while(ros::ok()){
//    int mapSz, poseSz;
//    buffer_.access.lock();
//    mapSz = buffer_.ogMapDeq.size();
//    poseSz = buffer_.poseDeq.size();
//    buffer_.access.unlock();

//    if(mapSz > 0 && poseSz > 0){
//      break;
//    }
//  }

//  ROS_INFO("Ready to recieve requests...");
//  std::unique_lock<std::mutex> lock1(goalAccess);
//  std::unique_lock<std::mutex> lock2(construction);

//  while(ros::ok()){
//    newGoal.wait(lock1);
//    TGlobalOrd goal = currentGoal_;

//    buildGoal.notify_one();

//    std::vector<TGlobalOrd> path;

//    int cnt(0);
//    while(++cnt < 5){
//      constructionComplete.wait(lock2);

//      path = gmap_.query(cspace, stat)

//    }



//  }

//}



void Simulator::prmThread() {
  cv::Mat ogMap;
  geometry_msgs::Pose robotPos;

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

  ROS_INFO("Ready to recieve requests...");
  std::unique_lock<std::mutex> lock(goalAccess);

  while(ros::ok()){
    //Wait until a new goal has been recieved
    newGoal.wait(lock);
    TGlobalOrd goal = currentGoal_;

    //Recieve new information from the world buffer
    consumeWorldInformation(ogMap, robotPos);

    //Update the reference for the localMap
    TGlobalOrd robotOrd = {robotPos.position.x, robotPos.position.y};
    ROS_INFO("Setting reference: {%.1f, %.1f}", robotOrd.x, robotOrd.y);
    gmap_.setReference(robotOrd);

    if(ogMap.empty()){
      //Something has gone wrong during image transmission,
      //skip execution for this goal and hope new data has arived
      //on the next go around
      ROS_ERROR("Empty OgMap");
      continue;
    }

    //Create colour copy of the OgMap
    cv::Mat colourMap;
    cv::cvtColor(ogMap, colourMap, CV_GRAY2BGR);

    //Expand the configuration space
    gmap_.expandConfigSpace(ogMap, robotDiameter_);


    //Validate both ordinates
    if(!gmap_.ordinateAccessible(ogMap, robotOrd)){
      ROS_ERROR(" Robot ordinates {%.1f, %.1f} are not accessible",
                robotOrd.x, robotOrd.y);
      continue;
    }

    if(!gmap_.ordinateAccessible(ogMap, goal)){
      ROS_ERROR(" Goal ordinates {%.1f, %.1f} are not accessible",
                goal.x, goal.y);
      continue;
    }

    //Start the build process
    ROS_INFO("Starting build: {%.1f, %.1f} to {%.1f, %.1f}",
             robotOrd.x, robotOrd.y, goal.x, goal.y);

    int cnt = 0;
    std::vector<TGlobalOrd> path;
    while(path.size() == 0){
      ROS_INFO("  Building nodes...");
      path = gmap_.build(ogMap, robotOrd, goal);

      if(cnt++ > 4){
        ROS_INFO("  Finding a path was more difficult than expected...");
        break;
      }
    }

    //Send infomration
    sendOverlay(colourMap, path);
    sendPath(path);
  }
}


bool Simulator::requestGoal(prm_sim::RequestGoal::Request &req, prm_sim::RequestGoal::Response &res)
{
  ROS_INFO("Goal request: x=%.1f, y=%.1f", (double)req.x, (double)req.y);

  //TODO: Check if Goal is within map space??
  //TODO: GOAL MUST BE FLOATING POINT
  currentGoal_.x = req.x;
  currentGoal_.y = req.y;

  newGoal.notify_one();
  res.ack = true;

  ROS_INFO("sending back response: [%d]", res.ack);
  return true;
}

void Simulator::consumeWorldInformation(cv::Mat &ogMap, geometry_msgs::Pose &robotPos){
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

void Simulator::sendOverlay(cv::Mat &colourMap, std::vector<TGlobalOrd> path){
  //Show the overlay of the prm
  gmap_.showOverlay(colourMap, path);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colourMap).toImageMsg();
  overlay_.publish(msg);
  ROS_INFO("Sent PRM overlay...");
}

void Simulator::sendPath(std::vector<TGlobalOrd> path){
  if(path.size() > 0){
    //Send the waypoints
    geometry_msgs::PoseArray posePath;
    for(auto const &waypoint: path)
    {
      geometry_msgs::Pose w;
      w.position.x = waypoint.x;
      w.position.y = waypoint.y;
      //TODO: do we need -> w.position.z = robotPos.position.z;

      posePath.poses.push_back(w);
    }

    path_.publish(posePath);
    ROS_INFO("Sent path information...");
  }
}
