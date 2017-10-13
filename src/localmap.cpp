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
  if(!inMap(start) || !inMap(end)){
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

void LocalMap::overlayPRM(cv::Mat &m, std::vector<std::pair<cv::Point, std::vector<cv::Point>>> prm){
  for(auto const &node: prm){
    //Draw circle to represent point
    cv::circle(m, node.first, 2, cv::Scalar(255,0,0),-1);

    //Connect neighbours
    for(auto const &neighbour: node.second){
      cv::line(m, node.first, neighbour, cv::Scalar(255,0,0),1);
    }
  }
}

void LocalMap::overlayPath(cv::Mat &m, std::vector<cv::Point> path){
  if(path.size() < 1){
    return; //No point drawing an empty path
  }

  cv::Point previousNode = path.at(0);

  for(auto const &node: path){
    //Draw circle to represent point
    cv::circle(m, node, 2, cv::Scalar(0,0,255),-1);

    //Connect to previous point
    cv::line(m, node, previousNode, cv::Scalar(0,0,255),1);
    previousNode = node;
  }
}

bool LocalMap::inMap(cv::Point p){
  return (p.y <= pixelMapSize_ && p.y >= 0) && (p.x <= pixelMapSize_ && p.x >= 0);
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

