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

#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const unsigned int MaxGraphDensity = 5;  /*!< The max amount of neighbours a vertex in the graph can have */
static const double MaxDistance = 2.5;          /*!< The max distance between two verticies in the graph */

GlobalMap::GlobalMap(double mapSize, double mapRes):
  graph_(Graph(MaxGraphDensity, MaxDistance)), lmap_(LocalMap(mapSize, mapRes))
{
  mapSize_ = mapSize;
  nextVertexId_ = 0;
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

std::vector<std::pair<cv::Point, cv::Point>> GlobalMap::constructPRM()
{
  std::vector<std::pair<cv::Point, cv::Point>> prm;
  std::map<vertex, edges> nodes = graph_.container();

  for(auto const &node: nodes){
    cv::Point pCurrent = lmap_.convertToPoint(reference_, vertexLUT_[node.first]);

    for(auto const &neighbour: node.second){
      cv::Point pNeighbour = lmap_.convertToPoint(reference_, vertexLUT_[neighbour.first]);

      //Ensure that we are only adding node pairs to the prm that are unique! (no two way connections)
      auto it = std::find_if(prm.begin(), prm.end(),
          [pCurrent, pNeighbour](const std::pair<cv::Point, cv::Point>& element){ return
            element.first == pNeighbour && element.second == pCurrent;});

      if(it == prm.end()){
        prm.push_back(std::make_pair(pCurrent, pNeighbour));
      }
    }
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



//m is greyscale
std::vector<TGlobalOrd> GlobalMap::build(cv::Mat &m, TGlobalOrd start, TGlobalOrd goal)
{
  vertex vStart, vGoal;     //Vertex representation of the global ords
  cv::Point pStart, pGoal;  //Pixel point representation of the ordinates
  std::vector<TGlobalOrd> path;

  //TODO: Expand config space based on robot size!!!!

  //If either does not exist as vertcies in graph already, we must perform some setup.
  if(!existsAsVertex(start) || !existsAsVertex(goal)){
    pStart = lmap_.convertToPoint(reference_, start);
    pGoal = lmap_.convertToPoint(reference_, goal);

    //First check that both points are accessible within the current map...
    if(!lmap_.isAccessible(m, pStart) || !lmap_.isAccessible(m, pGoal)){
      return path;
    }

    //Find or add the verticies to/in our graph
    vStart = findOrAdd(start);
    vGoal = findOrAdd(goal);
  }

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

  bool foundPath = false;
  unsigned int safetyCnt = 0;

  while(!foundPath){
    safetyCnt++;

    if(safetyCnt == 1000){
      //TODO: Determine if this is necessary?
      break;
    }

    TGlobalOrd randomOrd;
    //Generate random ords within the map space...
    std::default_random_engine generator(std::chrono::duration_cast<std::chrono::nanoseconds>
                                         (std::chrono::system_clock::now().time_since_epoch()).count());

    //TODO: generate points only in reference area??
    //TODO: Check if good.
    std::uniform_real_distribution<double> xDist(reference_.x - (mapSize_/2), reference_.x + (mapSize_/2));
    std::uniform_real_distribution<double> yDist(reference_.y - (mapSize_/2), reference_.y + (mapSize_/2));

    //round to 1 decimal place
    randomOrd.x = std::round((xDist(generator) * 10.0))/10.0;
    randomOrd.y = std::round((yDist(generator) * 10.0))/10.0;

    cv::Point pRand = lmap_.convertToPoint(reference_, randomOrd);

    //Only add ords that are accessible
    if(!lmap_.isAccessible(m, pRand)){
      continue;
    }

    //See if vertex exists in graph already otherwise, attempt to connect to other verticies
    vertex vRand = findOrAdd(randomOrd);
    connectToExistingNodes(m, vRand);

    //Check for path
    vPath = graph_.shortestPath(vStart, vGoal);
    if(vPath.size() > 0){
      path = convertPath(vPath);
      foundPath = true;
    }
  }

  return path;
}

bool GlobalMap::existsAsVertex(TGlobalOrd ord){
  for(auto &v: vertexLUT_){
    if(v.second.x == ord.x && v.second.y == ord.y){
      return true;
    }
  }

  return false;
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

void GlobalMap::setReference(const TGlobalOrd reference) {
  reference_.x = reference.x;
  reference_.y = reference.y;
}

void GlobalMap::setMapSize(double mapSize)
{
  mapSize_ = mapSize;
  lmap_.setMapSize(mapSize);
}
