/*! @file
 *
 *  @brief TODO
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "types.h"

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

private:
  ros::NodeHandle nh_;                      /*!< The handle of the ros node using this class */
  TWorldInfoBuffer &buffer_;                /*!< A shared global structure to update with world information */
  ros::ServiceServer reqGoal_;              /*!< Advertises a service '/request_goal' to set the goal */
  image_transport::ImageTransport overlay_; /*!< Publishes an overlay of the prm on top of the OgMap to /prm */
  ros::Publisher path_;                     /*!< Publishes the path between robot and goal on /path */
};

#endif // SIMULATOR_H
