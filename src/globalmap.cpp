/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#include "globalmap.h"

#include <algorithm>
#include <limits>
#include <iostream>
#include <string>
#include <math.h>
#include <iostream>

static const unsigned int MaxGraphDensity = 5; /*!< The max amount of neighbours a vertex in the graph can have */

GlobalMap::GlobalMap(double mapSize, double mapRes):
  graph_(Graph(MaxGraphDensity)), lmap_(LocalMap(mapSize, mapRes)), nextVertexId_(0)
{
}

std::vector<cv::Point> GlobalMap::convertPath(TGlobalOrd ref, std::vector<TGlobalOrd> path){
  std::vector<cv::Point> pointPath;
  for(auto const &ord: path){
    pointPath.push_back(lmap_.convertToPoint(ref, ord));
  }

  return pointPath;
}

std::vector<TGlobalOrd> GlobalMap::convertPath(std::vector<vertex> path){
  std::vector<TGlobalOrd> ordPath;

  for(auto const &v: path){
    //TODO: Slightly risky if its not in the LUT
    ordPath.push_back(vertexLUT_[v]);
  }

  return ordPath;
}

std::vector<std::pair<cv::Point, std::vector<cv::Point>>> GlobalMap::constructPRM(TGlobalOrd ref)
{
  std::vector<std::pair<cv::Point, std::vector<cv::Point>>> prm;
  std::map<vertex, edges> nodes = graph_.container();

  for(auto const &v: nodes){
    std::pair<cv::Point, std::vector<cv::Point>> pair;
    cv::Point vPoint = lmap_.convertToPoint(ref, vertexLUT_[v.first]);
    pair.first = vPoint;

    for(auto const &neighbour: v.second){
      pair.second.push_back(lmap_.convertToPoint(ref, vertexLUT_[neighbour.first]));
    }

    prm.push_back(pair);
  }

  return prm;
}

double distance(TGlobalOrd p1, TGlobalOrd p2){
  double a = std::abs(p2.x - p1.x);
  double b = std::abs(p2.y - p1.y);

  return std::sqrt(std::pow(a, 2) + std::pow(b, 2));
}

//Takes a colour image
void GlobalMap::showOverlay(cv::Mat &m, TGlobalOrd ref, std::vector<TGlobalOrd> path){
  std::vector<cv::Point> pPath = convertPath(ref, path);

  //Overlay onto map...
  lmap_.overlayPRM(m, constructPRM(ref));
  lmap_.overlayPath(m, pPath);
}

//m is greyscale
std::vector<TGlobalOrd> GlobalMap::build(cv::Mat &m, TGlobalOrd robotPos, TGlobalOrd goal)
{
  vertex vStart, vGoal;     //Vertex representation of the global ords
  cv::Point pStart, pGoal;  //Pixel point representation of the global ords

  //Check that coordiantes are on the global map, if not - add to network
  if(!existsAsVertex(robotPos)){
    vStart = nextVertexId();
    graph_.addVertex(vStart);
    vertexLUT_.insert(std::pair<vertex, TGlobalOrd>(vStart, robotPos));
  } else {
    lookup(robotPos, vStart); //it should exist, we just checked!
  }

  if(!existsAsVertex(goal)){
    vGoal = nextVertexId();
    graph_.addVertex(vGoal);
    vertexLUT_.insert(std::pair<vertex, TGlobalOrd>(vGoal, goal));
  } else {
    lookup(goal, vGoal); //it should exist, we just checked!
  }

  pStart = lmap_.convertToPoint(robotPos, robotPos);
  pGoal = lmap_.convertToPoint(robotPos, goal);

  //Check if there is already a path between the two points
  std::vector<vertex> vPath = graph_.shortestPath(vStart, vGoal);
  if(vPath.size() > 0){
    return convertPath(vPath);
  }

  //Check if we can add an edge between robot and goal before building
  if(lmap_.canConnect(m, pStart, pGoal)){

    //Will need to check its neighbour limit hasn't been reached
    graph_.addEdge(vStart, vGoal, distance(robotPos, goal));

    //Check if there is already a path between the two points
    std::vector<vertex> vPath = graph_.shortestPath(vStart, vGoal);
    if(vPath.size() > 0){
      return convertPath(vPath);
    }
  }



  //If none of the above
    //Generate random node within global map
    //Check its not already vertex
    //Add to graph
    //Attempt to connect edge to start and goal respectively
    //Calcualte path
    //Attempt to connect edge to other points in graph that aren't current, start, goal
    //Calculate path

  return std::vector<TGlobalOrd>();
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

bool GlobalMap::lookup(TGlobalOrd ord, vertex &v){
  for(auto &vert: vertexLUT_){
    if(vert.second.x == ord.x && vert.second.y == ord.y){
      v = vert.first;
      return true;
    }
  }

  return false;
}
