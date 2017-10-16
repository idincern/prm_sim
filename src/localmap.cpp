/*! @file
 *
 *  @brief Routines for obtaining information about a local map.
 *
 *  TODO: Better description?
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

static const cv::Scalar PrmColour = cv::Scalar(255,0,0);  /* Blue denotes prm colour */
static const cv::Scalar PathColour = cv::Scalar(0,0,255); /* Red denotes path colour */

LocalMap::LocalMap(double mapSize, double res): resolution_(res)
{
  pixelMapSize_ = (int) mapSize / res;
}

cv::Point LocalMap::convertToPoint(TGlobalOrd reference, TGlobalOrd ordinate){
  int convertedX;
  int convertedY;

  //With respect to the reference, determine which sector the ordinate is within
  if(ordinate.x > reference.x){
    convertedX = (pixelMapSize_ / 2) + (std::abs(std::round((ordinate.x - reference.x)/resolution_)));
  } else {
    convertedX = (pixelMapSize_ / 2) - (std::abs(std::round((ordinate.x - reference.x)/resolution_)));
  }

  if(ordinate.y > reference.y){
    convertedY = (pixelMapSize_ / 2) - (std::abs(std::round((ordinate.y - reference.y)/resolution_)));
  } else {
    convertedY = (pixelMapSize_ / 2) + (std::abs(std::round((ordinate.y - reference.y)/resolution_)));
  }

  return cv::Point(convertedX, convertedY);
}


void LocalMap::expandConfigSpace(cv::Mat &space, double robotDiameter){
  int pixDiameter = robotDiameter / resolution_;
  std::vector<cv::Point> pointsToExpand;

  //For each point on the map that isn't free space,
  //expand its boundary by the size of the robot diameter.
  for(int i = 0; i < space.rows; i++){
    for(int j = 0; j < space.cols; j++){
      if(space.at<uchar>(j,i) != 255){
        pointsToExpand.push_back({i, j});
      }
    }
  }

  //Simply draw a circle equal to the size of the robot at that point
  for(auto const &p: pointsToExpand){
    unsigned int inten = space.at<uchar>(p);

    //Even though this takes a radius, we double it (diameter) to ensure
    //there is some space between robot and obstacle
    cv::circle(space, p, pixDiameter, (inten, inten, inten), -1);
  }
}

void LocalMap::overlayPRM(cv::Mat &space, std::vector<std::pair<cv::Point, cv::Point>> prm){
  for(auto const &neighbours: prm){
    //Draw circles to represent points
    cv::circle(space, neighbours.first, 1, PrmColour,-1);
    cv::circle(space, neighbours.second, 1, PrmColour,-1);

    //Connect neighbours
    cv::line(space, neighbours.first, neighbours.second, PrmColour, 1);
  }
}

void LocalMap::overlayPath(cv::Mat &space, std::vector<cv::Point> path){
  if(path.size() < 1){
    return; //We don't want an out of bounds error from below
  }

  cv::Point previousNode = path.at(0);

  for(auto const &node: path){
    //Draw circle to represent point
    cv::circle(space, node, 2,PathColour,-1);

    //Connect to previous point
    cv::line(space, node, previousNode,PathColour,1);
    previousNode = node;
  }
}

bool LocalMap::canConnect(cv::Mat &cspace, cv::Point start, cv::Point end){
  //Do a bounds check
  if(!inMap(start) || !inMap(end)){
    return false;
  }

  //Iterate through each pixel between both points, checking that
  //each pixel is white = free space
  cv::LineIterator line(cspace, start, end);
  for(int i = 0; i < line.count; i++, line++){
    if(!isAccessible(cspace, line.pos())){
      return false;
    }
  }

  return true;
}

bool LocalMap::isAccessible(cv::Mat &cspace, cv::Point p){
  if(!inMap(p)){
    return false;
  }

  return (cspace.at<uchar>(p) == 255);
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

