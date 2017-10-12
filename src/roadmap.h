/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#ifndef ROADMAP_H
#define ROADMAP_H

#include "graph.h"
#include <map>
#include <utility>

typedef std::pair<unsigned int, unsigned int> TDimensions; /*!< X and Y limits of the map */

typedef struct
{
  unsigned int x; /*!< x coordinate within map */
  unsigned int y; /*!< y coordinate within map */
} TCoordinate;

class RoadMap
{
public:
  RoadMap(TDimensions d); //TODO: This might be an OgMap or something in future??

private:
  Graph graph_;                             /*!< A graph representation of the roadmap network */
  TDimensions dimensions_;                  /*!< The dimensions of the roadmap */
  std::map<vertex, TCoordinate> vertexLUT_; /*!< A look up table to convert a vertex to coordinate within map */
};

#endif // ROADMAP_H
