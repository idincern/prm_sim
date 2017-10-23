/*! @file
 *
 *  @brief A library of simple types shared amongst classes.
 *
 *  @author arosspope
 *  @date 13-10-2017
*/
#ifndef TYPES
#define TYPES

#include "nav_msgs/Odometry.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <deque>
#include <mutex>

struct TGlobalOrd
{
  double x;   /*!< x coordinate within global map (m) */
  double y;   /*!< y coordinate within global map (m) */

  bool operator== (const TGlobalOrd &o1){
    return (this->x == o1.x && this->y == o1.y);
  }
};

struct TWorldDataBuffer /*!< Used as a container for map information */
{
  std::deque<geometry_msgs::Pose> poseDeq;  /*!< A queue of robot poses */
  std::deque<cv::Mat> ogMapDeq;             /*!< A queue of OgMaps */
  std::mutex access;                        /*!< Mutex to control access to the buffer */
};

#endif // TYPES

