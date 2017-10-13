/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#ifndef LOCALMAP_H
#define LOCALMAP_H

#include "graph.h"
#include "types.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <map>
#include <utility>

class LocalMap
{
public:
  //TODO: Inputs robot size for effective cfg space?
  //mapSize in meters
  LocalMap(double mapSize, double res);

  //Converts global ordinates (in m) to local ords
  //Given the robots global position, convert the point global position to pixel point
  cv::Point convertToPoint(TGlobalOrd reference, TGlobalOrd ordinate);

  //Given a local map, determine if we can connect two ordinates
  //Assumes m is the size defined by map size
  bool canConnect(cv::Mat &m, cv::Point start, cv::Point end);

  //checks if a point is in the known map space
  bool inMap(cv::Point p);

  //Draw prm overlay on local map
  //For each point in map, draw a circle and a line to all the other points
  void overlayPRM(cv::Mat &m, std::vector<std::pair<cv::Point, std::vector<cv::Point>>> prm);

  //Overlay a red path on the map, it assumes the points are in order and connected to eachother
  void overlayPath(cv::Mat &m, std::vector<cv::Point> path);

  void setMapSize(double mapSize);

  void setResolution(double resolution);

private:
  double resolution_;         /*!< Will specify the amount of pixels per meter */
  unsigned int pixelMapSize_; /*!< The total mapSize (square maps/images only) in pixels */

};

#endif // LOCALMAP_H
