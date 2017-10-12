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
#include <opencv2/imgproc/imgproc.hpp>

LocalMap::LocalMap(double mapSize, double res): resolution_(res)
{
  pixelMapSize_ = (int) mapSize / res;
}

bool LocalMap::canConnect(cv::Mat &m, cv::Point start, cv::Point end){
  //Do a bounds check
  if((start.y > pixelMapSize_ || start.y < 0) ||
     (start.x > pixelMapSize_ || start.x < 0)){
    return false;
  }

  if((end.y > pixelMapSize_ || end.y < 0) ||
     (end.x > pixelMapSize_ || end.x < 0)){
    return false;
  }

  //Iterate through each pixel between both points, checking that
  //each pixel is white = free space
  cv::LineIterator line(m, start, end);
  for(int i = 0; i < line.count; i++, line++){
    if(m.at<uchar>(line.pos()) != 255){
      return false;
    }
  }

  return true;
}

void LocalMap::setMapSize(double mapSize)
{
  pixelMapSize_ = (int) mapSize / resolution_;
}

void LocalMap::setResolution(double resolution)
{
  resolution_ = resolution;
}

//Drawing lines: https://stackoverflow.com/questions/28780947/opencv-create-new-image-using-cvmat

