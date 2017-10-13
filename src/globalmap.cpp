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
#include <random>
#include <iostream>
#include <thread>
#include <chrono>

static const unsigned int MaxGraphDensity = 5; /*!< The max amount of neighbours a vertex in the graph can have */

GlobalMap::GlobalMap(double mapSize, double mapRes):
  graph_(Graph(MaxGraphDensity)), lmap_(LocalMap(mapSize, mapRes)), nextVertexId_(0)
{
  reference_.x = 0;
  reference_.y = 0;
}

std::vector<cv::Point> GlobalMap::convertPath(std::vector<TGlobalOrd> path){
  std::vector<cv::Point> pointPath;
  for(auto const &ord: path){
    pointPath.push_back(lmap_.convertToPoint(reference_, ord));
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

std::vector<std::pair<cv::Point, std::vector<cv::Point>>> GlobalMap::constructPRM()
{
  std::vector<std::pair<cv::Point, std::vector<cv::Point>>> prm;
  std::map<vertex, edges> nodes = graph_.container();

  for(auto const &v: nodes){
    std::pair<cv::Point, std::vector<cv::Point>> pair;
    cv::Point vPoint = lmap_.convertToPoint(reference_, vertexLUT_[v.first]);
    pair.first = vPoint;

    for(auto const &neighbour: v.second){
      pair.second.push_back(lmap_.convertToPoint(reference_, vertexLUT_[neighbour.first]));
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
void GlobalMap::showOverlay(cv::Mat &m, std::vector<TGlobalOrd> path){
  std::vector<cv::Point> pPath = convertPath(path);

  //Overlay onto map...
  lmap_.overlayPRM(m, constructPRM());
  lmap_.overlayPath(m, pPath);
}

void GlobalMap::setReference(const TGlobalOrd reference)
{
  reference_.x = reference.x;
  reference_.y = reference.y;
}

//Given an existing node, attempt to connect to other points within the network
void GlobalMap::connectToExistingNodes(cv::Mat &m, vertex node){
  cv::Point currentPos = lmap_.convertToPoint(reference_, vertexLUT_[node]);

  for(auto const &vertex: vertexLUT_){
    if(vertex.first == node){
      continue;
    }

    cv::Point vPoint = lmap_.convertToPoint(reference_, vertex.second);
    if(lmap_.canConnect(m, currentPos, vPoint)){
      graph_.addEdge(node, vertex.first, distance(vertexLUT_[node], vertex.second));
    }
  }
}

vertex GlobalMap::findOrAdd(TGlobalOrd ordinate){
  vertex v;
  if(existsAsVertex(ordinate)){
    lookup(ordinate, v);
  } else {
    v = nextVertexId();
    graph_.addVertex(v);
    vertexLUT_.insert(std::make_pair(v, ordinate));
  }

  return v;
}

//m is greyscale
std::vector<TGlobalOrd> GlobalMap::build(cv::Mat &m, TGlobalOrd start, TGlobalOrd goal)
{
  //Vertex representation of the global ords
  vertex vStart = findOrAdd(start);
  vertex vGoal = findOrAdd(goal);

  //Pixel point representation of the ordinates
  cv::Point pStart = lmap_.convertToPoint(reference_, start);
  cv::Point pGoal = lmap_.convertToPoint(reference_, goal);

  //Check if there is already a path between the two points
  std::vector<vertex> vPath = graph_.shortestPath(vStart, vGoal);
  if(vPath.size() > 0){
    return convertPath(vPath);
  }

  //Check if we can add an edge between start and goal before building process
  connectToExistingNodes(m, vStart);
  connectToExistingNodes(m, vGoal);
  vPath = graph_.shortestPath(vStart, vGoal);
  if(vPath.size() > 0){
    return convertPath(vPath);
  }

  for(int i=0; i < 1000; i++){
    TGlobalOrd randomOrd;
    //Generate random nodes...
    std::default_random_engine generator(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::uniform_real_distribution<double> distribution(-20, 20);

    //round to 1 decimal place
    randomOrd.x = std::round((distribution(generator) * 10.0))/10.0;
    randomOrd.y = std::round((distribution(generator) * 10.0))/10.0;

    vertex v = findOrAdd(randomOrd);
    connectToExistingNodes(m, v);
    vPath = graph_.shortestPath(vStart, vGoal);
    if(vPath.size() > 0){
      return convertPath(vPath);
    }
  }


  //Attempt to connect to other nodes

  //If node is grey we can add, otherwise, if its black, then no!


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
