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
#ifndef WORLDRETRIEVE_H
#define WORLDRETRIEVE_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/image_encodings.h"
#include "types.h"

class WorldRetrieve
{
public:
  /*! @brief Constructor for WorldRetrieve.
   *
   *  @param nh The handle of the ros node using this class
   *  @param buffer A reference to a shared world information buffer
   */
  WorldRetrieve(ros::NodeHandle nh, TWorldInfoBuffer &buffer);
private:
  ros::NodeHandle nh_;        /*!< The handle of the ros node using this class */
  ros::Subscriber odom_;      /*!< A subscription to the /odom topic */
  ros::Subscriber ogmap_;     /*!< A subscription to the /map_image/full topic */
  TWorldInfoBuffer &buffer_;  /*!< A shared global structure to update with world information */

  void odomCallBack(const nav::msgs::OdometryConstPtr &msg);

  void ogMapCallBack(const sensor_msgs::ImageConstPtr &msg);

};

#endif // WORLDRETRIEVE_H
