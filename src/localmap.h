/*! @file
 *
 *  @brief Routines for obtaining information about a local map.
 *
 *  TODO: Better description?
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#ifndef LOCALMAP_H
#define LOCALMAP_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>

#include "types.h"

class LocalMap
{
public:
  //TODO: Inputs robot size for effective cfg space?
  /*! @brief Constructor for LocalMap.
   *
   *  @param mapSize The size of the overall map in meters (square maps only).
   *  @param res The resolution of the local maps provided to this object.
   */
  LocalMap(double mapSize, double res);

  /*! @brief Converts a Global coordinate to a pixel coordinate
   *
   *  @param reference The reference position to base our conversion off.
   *                   This is usually the robot's position.
   *  @param ordinate The coordinate to convert.
   *  @return Point The converted point.
   */
  cv::Point convertToPoint(TGlobalOrd reference, TGlobalOrd ordinate);

  /*! @brief Given a map, determine if two points can be connected.
   *
   *  This method determines if there are any obstacles between start and
   *  end. The colour white in a map denotes known 'free space'. Anything
   *  else is unknown or blocked.
   *
   *  @param m The map to look within. Note, this must be a greyscale image!
   *  @param start The starting position.
   *  @param end The ending position.
   *  @return bool - TRUE if there is nothing blocking the path between
   *                 start and end.
   */
  bool canConnect(cv::Mat &m, cv::Point start, cv::Point end);

  /*! @brief Checks if a point is within the known local map.
   *
   *  @param p The point to test for its place within the map.
   *  @return bool - TRUE if it is within the map.
   */
  bool inMap(cv::Point p);

  /*! @brief Checks if a point is within free space.
   *
   *  @param m A greyscale map image.
   *  @param p The point to test for within the map.
   *  @return bool - TRUE if the point is accessible.
   */
  bool isAccessible(cv::Mat &m, cv::Point p);

  /*! @brief Draws a Probablistic Road Map onto an existing map.
   *
   *  Will draw a blue circle to represent a node, and blue lines to represent
   *  its connections to other nodes within the PRM.
   *
   *  @param m The map to overlay the PRM on top of. It is assumed that this
   *           is a color enabled image (not greyscale).
   *  @param prm A network of pixel points, and its connection to other points.
   */
  void overlayPRM(cv::Mat &m, std::vector<std::pair<cv::Point, std::vector<cv::Point>>> prm);

  /*! @brief Draws a path onto an existing map.
   *
   *  Will draw a red circle to represent a node, and blue lines to represent
   *  its connections to the next node in the path. Primarily used for showing
   *  a path between a start and end goal.
   *
   *  @param m The map to overlay the PRM on top of. It is assumed that this
   *           is a color enabled image (not greyscale).
   *  @param path An ordered vector representing a path between points.
   */
  void overlayPath(cv::Mat &m, std::vector<cv::Point> path);

  /*! @brief Setter for updating the map size (square maps only).
   *
   *  @param mapSize The size of the overall map in meters.
   */
  void setMapSize(double mapSize);

  /*! @brief Setter for updating map resolution.
   *
   *  @param resolution The resolution of the local maps provided to this object.
   */
  void setResolution(double resolution);

private:
  double resolution_;         /*!< Will specify the amount of pixels per meter */
  unsigned int pixelMapSize_; /*!< The total mapSize (square maps/images only) in pixels */

};

#endif // LOCALMAP_H
