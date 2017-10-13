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
  std::vector<TGlobalOrd> build(cv::Mat &m, TGlobalOrd robotPos, TGlobalOrd goal);

  //Overlays internal PRM and path onto a colour image
  void showOverlay(cv::Mat &m, TGlobalOrd ref, std::vector<TGlobalOrd> path);

private:
  Graph graph_;                             /*!< A graph representation of the roadmap network */
  LocalMap lmap_;                           /*!< An object for interacting with the ogMap provided to this object */
  std::map<vertex, TGlobalOrd> vertexLUT_;  /*!< A look up table to convert a vertex to coordinate within map */
  vertex nextVertexId_;                     /*!< Used for generating unique vertex ids for coordiantes... TODO: make atomic?? */

  //returns true if ordinates are in vertexLUT_;
  bool existsAsVertex(TGlobalOrd ord);

  //Gets the next available vertex id
  vertex nextVertexId();

  //Lookups vertex id in vertexLUT_, will return false if nothing found
  bool lookup(TGlobalOrd ord, vertex &v);

  //Converts a path of globalords to pixel points
  std::vector<cv::Point> convertPath(TGlobalOrd ref, std::vector<TGlobalOrd> path);

  //Converts a path of vertexes to globalOrds
  std::vector<TGlobalOrd> convertPath(std::vector<vertex> path);

  std::vector<std::pair<cv::Point, std::vector<cv::Point>>> constructPRM(TGlobalOrd ref);
};

#endif // GLOBALMAP_H
