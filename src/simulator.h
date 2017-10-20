/*! @file
 *
 *  @brief TODO
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
#include "types.h"
#include "prmplanner.h"
#include "prm_sim/RequestGoal.h"

//TODO: Should this be called simulator??
class Simulator
{
public:
  /*! @brief Constructor for Simulator.
   *
   *  @param nh The handle of the ros node using this class
   *  @param buffer A reference to a shared world information buffer
   */
  Simulator(ros::NodeHandle nh, TWorldInfoBuffer &buffer);

  //Does the prm stuff!
  void prmThread(void);

private:
  ros::NodeHandle nh_;                      /*!< The handle of the ros node using this class */
  TWorldInfoBuffer &buffer_;                /*!< A shared global structure to update with world information */
  ros::ServiceServer reqGoal_;              /*!< Advertises a service '/request_goal' to set the goal */

  image_transport::ImageTransport it_;      /*!< Transport mechanism for images */
  image_transport::Publisher overlay_;      /*!< Publishes an overlay of the prm on top of the OgMap to /prm */
  ros::Publisher path_;                     /*!< Publishes the path between robot and goal on /path */

  PrmPlanner gmap_;
  double robotDiameter_; //TODO

  //TODO: Do these need to be atomic?
  //TODO: These could be internal to the thread....
  TGlobalOrd currentGoal_;  /*!< Current goal to be moving towards */
  //condition var to wait on new goals


  bool requestGoal(prm_sim::RequestGoal::Request &req, prm_sim::RequestGoal::Response &res);
  void consumeWorldInformation(cv::Mat &ogMap, geometry_msgs::Pose &robotPos);
  void sendOverlay(cv::Mat &colourMap, std::vector<TGlobalOrd> path);
  void sendPath(std::vector<TGlobalOrd> path);
};

#endif // SIMULATOR_H
