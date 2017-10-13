/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#ifndef GLOBALMAP_H
#define GLOBALMAP_H

#include <map>
#include <utility>

#include "localmap.h"
#include "graph.h"
#include "types.h"


class GlobalMap
{
public:
  //Map size in meters
  GlobalMap(double mapSize, double mapRes);

  //Returns a path of ordinates between the two ords
  std::vector<TGlobalOrd> build(cv::Mat &m, TGlobalOrd start, TGlobalOrd goal);

  //Overlays internal PRM and path onto a colour image
  void showOverlay(cv::Mat &m, std::vector<TGlobalOrd> path);

  //TODO: FUNCTION that excepts a mat and expands the size of any grey or black value by the size of the robot

  void setReference(const TGlobalOrd reference);

private:
  Graph graph_;                             /*!< A graph representation of the roadmap network */
  LocalMap lmap_;                           /*!< An object for interacting with the ogMap provided to this object */
  std::map<vertex, TGlobalOrd> vertexLUT_;  /*!< A look up table to convert a vertex to coordinate within map */ //TODO: Rename as network??
  vertex nextVertexId_;                     /*!< Used for generating unique vertex ids for coordiantes... TODO: make atomic?? */
  TGlobalOrd reference_;                    /*!< Reference ordinate for the local map, this is usually the robot position */

  //TODO: MAKE REF an internal variable?

  //returns true if ordinates are in vertexLUT_;
  bool existsAsVertex(TGlobalOrd ord);

  //Gets the next available vertex id
  vertex nextVertexId();

  //Lookups vertex id in vertexLUT_, will return false if nothing found
  bool lookup(TGlobalOrd ord, vertex &v);

  //Converts a path of globalords to pixel points
  std::vector<cv::Point> convertPath(std::vector<TGlobalOrd> path);

  //Converts a path of vertexes to globalOrds
  std::vector<TGlobalOrd> convertPath(std::vector<vertex> path);

  //Consruct a PRM network for overlay later
  std::vector<std::pair<cv::Point, std::vector<cv::Point>>> constructPRM();

  //connect node to all existing verticies
  void connectToExistingNodes(cv::Mat &m, vertex node);

  //Attempts to find an ordinate as existing in network, if not make a new entry and return
  vertex findOrAdd(TGlobalOrd ordinate);
};

#endif // GLOBALMAP_H
