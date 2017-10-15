/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#include "globalmap.h"

#include <iostream>
#include <math.h>
#include <random>
#include <thread>
#include <chrono>

static const unsigned int MaxGraphDensity = 5;  /*!< The max amount of neighbours a vertex in the graph can have */
static const double MaxDistance = 4;          /*!< The max distance between two verticies in the graph (2.5) */

GlobalMap::GlobalMap(double mapSize, double mapRes, double robotDiameter):
  graph_(Graph(MaxGraphDensity, MaxDistance)), lmap_(LocalMap(mapSize, mapRes))
{
  mapSize_ = mapSize;
  nextVertexId_ = 0;
  reference_.x = 0;
  reference_.y = 0;
  robotDiameter_ = robotDiameter;
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
    ordPath.push_back(vertexLUT_[v]);
  }

  return ordPath;
}

std::vector<std::pair<cv::Point, cv::Point>> GlobalMap::constructPRM()
{
  std::vector<std::pair<cv::Point, cv::Point>> prm;
  std::map<vertex, edges> nodes = graph_.container();

  //For each vertex in our internal graph, create a pair of points
  //between itself and all its neighbours
  for(auto const &node: nodes){
    cv::Point pCurrent = lmap_.convertToPoint(reference_, vertexLUT_[node.first]);

    //It has no neighbours, we must still add it to the prm though
    if(node.second.size() == 0){
      prm.push_back(std::make_pair(pCurrent, pCurrent));
    }

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


void GlobalMap::connectNodes(cv::Mat &m){
  //entry.first = vertex
  //entry.second = ord
  for(auto const &entry: vertexLUT_){
    std::vector<vertex> checked;
    checked.push_back(entry.first);

    int cnt(0);

    std::vector<vertex> withinRange;
    for(auto const &subentry: vertexLUT_){
      if(graph_.canConnect(subentry.first, distance(entry.second, subentry.second))){
        withinRange.push_back(subentry.first);
      }
    }

    for(auto const &n: withinRange){
      TGlobalOrd ordCurrent = vertexLUT_[entry.first];
      TGlobalOrd ordN = vertexLUT_[n];

      cv::Point pCurrent = lmap_.convertToPoint(reference_, ordCurrent);
      cv::Point pN = lmap_.convertToPoint(reference_, ordN);

      if(lmap_.canConnect(m,pCurrent,pN)){
        if(graph_.addEdge(entry.first, n, distance(ordCurrent, ordN))){
          cnt++;
        }
      }

      if(cnt > MaxGraphDensity){
        break;
      }
    }
  }
}

//Given an existing node, attempt to connect to other points within the network
//TODO: Pass in ord instead
void GlobalMap::connectToExistingNodes(cv::Mat &m, vertex vToAdd){
  TGlobalOrd ordToAdd = vertexLUT_[vToAdd];
  cv::Point pToAdd = lmap_.convertToPoint(reference_, vertexLUT_[vToAdd]);

  for(auto const &vertex: vertexLUT_){
    if(vertex.first == vToAdd){
      continue;
    }

    cv::Point pVertex = lmap_.convertToPoint(reference_, vertex.second);
    if(lmap_.canConnect(m, pToAdd, pVertex)){
      graph_.addEdge(vToAdd, vertex.first, distance(ordToAdd, vertex.second));
    }
  }
}

//m is greyscale
std::vector<TGlobalOrd> GlobalMap::build(cv::Mat &m, TGlobalOrd start, TGlobalOrd goal)
{
  vertex vStart, vGoal;     //Vertex representation of the global ords
  cv::Point pStart, pGoal;  //Pixel point representation of the ordinates
  std::vector<TGlobalOrd> path;

  //Expand config space based on the robot's diameter
  lmap_.expandConfigSpace(m, robotDiameter_);

  //If either does not exist as vertcies in graph already, we must perform some setup.
  if(!existsAsVertex(start) || !existsAsVertex(goal)){
    pStart = lmap_.convertToPoint(reference_, start);
    pGoal = lmap_.convertToPoint(reference_, goal);

    //First check that both points are accessible within the current map...
    if(!lmap_.isAccessible(m, pStart) || !lmap_.isAccessible(m, pGoal)){
      return path; //if not, return empty path
    }

    //Find or add the verticies to/in our graph
    vStart = findOrAdd(start);
    vGoal = findOrAdd(goal);
  }

  connectNodes(m);

  //Check if there is already a path between the two points
  std::vector<vertex> vPath = graph_.shortestPath(vStart, vGoal);
  if(vPath.size() > 0){
    return convertPath(vPath);
  }

  unsigned int nodeCnt(0);

  while(nodeCnt < 50){
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
    nodeCnt++;
  }

  //Check for path
  connectNodes(m);

  std::cout << "calculating path..." << std::endl;

//  vPath = graph_.shortestPath(vStart, vGoal);
//  if(vPath.size() > 0){
//    return convertPath(vPath);
//  }

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
