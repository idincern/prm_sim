/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#include "localmap.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

LocalMap::LocalMap(): map_(cv::Mat())
{
}

void LocalMap::setMap(cv::Mat m)
{
  //m.copyTo(map_); -> Issue, perhaps we can only pass in mat as local variable
}

