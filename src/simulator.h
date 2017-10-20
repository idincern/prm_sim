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
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <opencv2/opencv.hpp>
#include <atomic>
#include <image_transport/image_transport.h>

#include "ros/ros.h"
#include "prm_sim/RequestGoal.h"
#include "prmplanner.h"
#include "types.h"

typedef struct {
  cv::Mat cspace;
  cv::Mat prmOnSpace;
  std::mutex access;
} TMapContainer;


class Simulator
{
public:
  /*! @brief Constructor for Simulator.
   *
   *  @param nh The handle of the ros node using this class
   *  @param buffer A reference to a shared world information buffer. This buffer should
   *                be populated by another thread.
   */
  Simulator(ros::NodeHandle nh, TWorldInfoBuffer &buffer);

  /*! @brief Creates a path between robot and goal using PRM planner.
   *
   *  This thread waits on a goal then attempts to plan a path between
   *  the robot's last known position and the goal.
   *
   *  @note Whilst the planner is building the network, multiple goal requests
   *        are ignored.
   */
  void plannerThread(void);

private:
  ros::NodeHandle nh_;                      /*!< The handle of the ros node using this class */
  image_transport::ImageTransport it_;      /*!< Transport mechanism for images */
  ros::ServiceServer reqGoal_;              /*!< Advertises a service '/request_goal' to set the goal */
  image_transport::Publisher overlay_;      /*!< Publishes an overlay of the prm on top of the OgMap to /prm */
  ros::Publisher path_;                     /*!< Publishes the path between robot and goal on /path */

  TWorldInfoBuffer &buffer_;                /*!< A shared global structure that gets updated with world information */
  PrmPlanner planner_;                      /*!< The LD-PRM planner for path finding */

  double robotDiameter_;                    /*!< Diameter of the robot in meters */
  TGlobalOrd goal_;                         /*!< The current goal for the robot to reach */
  cv::Mat cspace_;                          /*!< The current configuration space (greyscale) */
  cv::Mat prmOnSpace_;                      /*!< An image of the last known prm/path overlayed onto the cspace */
  geometry_msgs::Pose robotPos_;            /*!< The current robot position */

  //TODO: In here? condition var to wait on new goals


  bool requestGoal(prm_sim::RequestGoal::Request &req, prm_sim::RequestGoal::Response &res);
  void consumeWorldData(cv::Mat &ogMap, geometry_msgs::Pose &robotPos);
  void sendOverlay(cv::Mat &colourMap, std::vector<TGlobalOrd> path);
  void sendPath(std::vector<TGlobalOrd> path);
  void waitForWorldData();
};

#endif // SIMULATOR_H
