/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#ifndef GLOBALMAP_H
#define GLOBALMAP_H

#include "graph.h"
#include "types.h"
#include <map>
#include <utility>

class GlobalMap
{
public:
  GlobalMap();
  GlobalMap(double mapWidth, double mapLength, double resolution); //TODO: This might be an OgMap or something in future??

  void build(TGlobalOrd start, TGlobalOrd goal);

private:
  Graph graph_;                             /*!< A graph representation of the roadmap network */
  double mWidth_, mLength_, resolution_;    /*!< Max width and length of the global map */
  std::map<vertex, TGlobalOrd> vertexLUT_;  /*!< A look up table to convert a vertex to coordinate within map */
  vertex nextVertexId_;                     /*!< Used for generating unique vertex ids for coordiantes... TODO: make atomic?? */

  //returns true if ordinates are in vertexLUT_;
  bool existsAsVertex(TGlobalOrd ord);

  //Gets the next available vertex id
  vertex nextVertexId();
};

#endif // GLOBALMAP_H
