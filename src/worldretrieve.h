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

#include <image_transport/image_transport.h>
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
   *  @param buffer A reference to a shared world data buffer
   */
  WorldRetrieve(ros::NodeHandle nh, TWorldDataBuffer &buffer);

private:
  ros::NodeHandle nh_;                /*!< The handle of the ros node using this class */
  ros::Subscriber odom_;              /*!< A subscription to the /odom topic */
  image_transport::Subscriber ogmap_; /*!< A subscription to the /map_image/full topic */
  TWorldDataBuffer &buffer_;          /*!< A shared global structure to update with world information */

  /*! @brief Call back for receiving robot poses.
   *
   *  Upon triger, this function will put the robot's pose into the internal
   *  buffer for consumption by another thread. it is important to note
   *  that old data in the buffer will be overwritten if not consumed
   *  in a timely manner.
   *
   *  @param msg A ros msg containing the pose.
   */
  void odomCallBack(const nav_msgs::OdometryConstPtr &msg);

  /*! @brief Call back for receiving ogmap images.
   *
   *  Upon triger, this function will put an image into the internal
   *  buffer for consumption by another thread. it is important to note
   *  that old data in the buffer will be overwritten if not consumed
   *  in a timely manner.
   *
   *  @param msg A ros msg containing the ogmap.
   *
   *  @note Colour images will be converted to greyscale.
   */
  void ogMapCallBack(const sensor_msgs::ImageConstPtr &msg);

};

#endif // WORLDRETRIEVE_H
