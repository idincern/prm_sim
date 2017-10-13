/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#include "localmap.h"

#include <math.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

LocalMap::LocalMap(double mapSize, double res): resolution_(res)
{
  pixelMapSize_ = (int) mapSize / res;
}

cv::Point LocalMap::convertToPoint(TGlobalOrd reference, TGlobalOrd ordinate){
  //TODO: FACTOR IN SIZE OF ROBOT???
  int convertedX;
  int convertedY;

  //With respect to the reference, determine which sector the ordinate is within
  if(ordinate.x > reference.x){
    convertedX = (pixelMapSize_ / 2) + (std::abs(std::round(ordinate.x - reference.x))/resolution_);
  } else {
    convertedX = (pixelMapSize_ / 2) - (std::abs(std::round(ordinate.x - reference.x))/resolution_);
  }

  if(ordinate.y > reference.y){
    convertedY = (pixelMapSize_ / 2) - (std::abs(std::round(ordinate.y - reference.y))/resolution_);
  } else {
    convertedY = (pixelMapSize_ / 2) + (std::abs(std::round(ordinate.y - reference.y))/resolution_);
  }

  return cv::Point(convertedX, convertedY);
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

