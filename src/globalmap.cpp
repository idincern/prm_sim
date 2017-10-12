/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#include "globalmap.h"
#include "graph.h"

#include <algorithm>
#include <limits>
#include <iostream>
#include <string>

static const unsigned int MaxGraphDensity = 5; /*!< The max amount of neighbours a vertex in the graph can have */

GlobalMap::GlobalMap(): graph_(Graph(MaxGraphDensity)), nextVertexId_(0)
{
  //TODO: INIT PROPERLY
}

void GlobalMap::build(TGlobalOrd start, TGlobalOrd goal)
{
  //Check that coordiantes are on the global map

  if(!existsAsVertex(start)){
    vertexLUT_.insert(std::pair<vertex, TGlobalOrd>(nextVertexId(), start));
  }

  if(!existsAsVertex(goal)){
    vertexLUT_.insert(std::pair<vertex, TGlobalOrd>(nextVertexId(), goal));
  }

  //Check if there is a path between the two points

  //Check if we can add an edge between the two points

  //If none of the above
    //Generate random node within global map
    //Check its not already vertex
    //Add to graph
    //Attempt to connect edge to start and goal respectively
    //Calcualte path
    //Attempt to connect edge to other points in graph that aren't current, start, goal
    //Calculate path
}

bool GlobalMap::existsAsVertex(TGlobalOrd ord){
  for(auto &v: vertexLUT_){
    if(v.second.x == ord.x && v.second.y == ord.y){
      return true;
    }
  }

  return false;
}

vertex GlobalMap::nextVertexId(){
  vertex temp = nextVertexId_;
  nextVertexId_++;
  return temp;
}
