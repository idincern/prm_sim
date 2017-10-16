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
static const double MaxDistance = 2.5;          /*!< The max distance between two verticies in the graph (2.5) */

GlobalMap::GlobalMap(double mapSize, double mapRes, double robotDiameter):
  graph_(Graph(MaxGraphDensity)), lmap_(LocalMap(mapSize, mapRes))
{
  mapSize_ = mapSize;
  nextVertexId_ = 0;
  reference_.x = 0;
  reference_.y = 0;
  robotDiameter_ = robotDiameter;
}

std::vector<TGlobalOrd> GlobalMap::build(cv::Mat &space, TGlobalOrd start, TGlobalOrd goal)
{
  vertex vStart, vGoal;     //Vertex representation of the global ords
  cv::Point pStart, pGoal;  //Pixel point representation of the ordinates
  std::vector<TGlobalOrd> path;

  //Expand config space based on the robot's diameter
  lmap_.expandConfigSpace(space, robotDiameter_);

  //If either does not exist as vertcies in graph already, we must perform some setup.
  if(!existsAsVertex(start) || !existsAsVertex(goal)){
    pStart = lmap_.convertToPoint(reference_, start);
    pGoal = lmap_.convertToPoint(reference_, goal);

    //First check that both points are accessible within the current map...
    if(!lmap_.isAccessible(space, pStart) || !lmap_.isAccessible(space, pGoal)){
      return path; //if not, return empty path
    }
  }

  unsigned int nodeCnt(0);

  //TODO: Dynamically work out nodes?
  while(nodeCnt < 200){
    TGlobalOrd randomOrd;
    //Generate random ords within the map space...
    std::default_random_engine generator(std::chrono::duration_cast<std::chrono::nanoseconds>
                                         (std::chrono::system_clock::now().time_since_epoch()).count());

    std::uniform_real_distribution<double> xDist(reference_.x - (mapSize_/2), reference_.x + (mapSize_/2));
    std::uniform_real_distribution<double> yDist(reference_.y - (mapSize_/2), reference_.y + (mapSize_/2));

    //round to 1 decimal place
    randomOrd.x = std::round((xDist(generator) * 10.0))/10.0;
    randomOrd.y = std::round((yDist(generator) * 10.0))/10.0;

    cv::Point pRand = lmap_.convertToPoint(reference_, randomOrd);

    //Only add ords that are accessible
    if(!lmap_.isAccessible(space, pRand)){
      continue;
    }

    //See if vertex exists in graph already otherwise, attempt to connect to other verticies
    vertex vRand = findOrAdd(randomOrd);
    nodeCnt++;

    connectNodes(space, true);
  }

  //Find or add the verticies to/in our graph
  //TODO: HAve no distance constraint when adding these guys
  vStart = findOrAdd(start);
  vGoal = findOrAdd(goal);

  //Check for path
  connectNode(space, vStart, false);
  connectNode(space, vGoal, false);
  //connectNodes(m, true);

  std::vector<vertex> vPath = graph_.shortestPath(vStart, vGoal);
  if(vPath.size() > 0){
    return optimisePath(space, toOrdPath(vPath));
    //return convertPath(vPath);
  }

  return path;
}

void GlobalMap::showOverlay(cv::Mat &space, std::vector<TGlobalOrd> path){
  std::vector<cv::Point> pPath = toPointPath(path);

  //Overlay onto map...
  lmap_.overlayPRM(space, getPRM());
  lmap_.overlayPath(space, pPath);
}

std::vector<TGlobalOrd> GlobalMap::optimisePath(cv::Mat &cspace, std::vector<TGlobalOrd> path){
  std::vector<TGlobalOrd> optPath;

  if(path.size() == 0){
    return optPath; //No path to optimise return empty path
  }

  //Start with the first node
  optPath.push_back(path.at(0));

  //While the goal is not in the optimised path
  while(std::find(optPath.begin(), optPath.end(), path.back()) == optPath.end()){
    TGlobalOrd ordCurr = optPath.back();
    cv::Point pCurrent = lmap_.convertToPoint(reference_, ordCurr);

    //Starting at the end of the path and moving backwards, determine
    //if we can directly connect to the current ordinate
    for(unsigned i = path.size(); i-- > 0;){
      if(path[i] == ordCurr){
        break; //We have reached the current node
      }

      cv::Point pTest = lmap_.convertToPoint(reference_, path[i]);
      if(lmap_.canConnect(cspace, pCurrent, pTest)){
        optPath.push_back(path[i]);
        break; //We have found the latest node to push back to
      }
    }
  }

  return optPath;
}

std::vector<std::pair<cv::Point, cv::Point>> GlobalMap::getPRM()
{
  std::vector<std::pair<cv::Point, cv::Point>> prm;
  std::map<vertex, edges> nodes = graph_.container();

  //For each vertex in our internal graph, create a pair of points
  //between itself and all its neighbours
  for(auto const &node: nodes){
    cv::Point pCurrent = lmap_.convertToPoint(reference_, network_[node.first]);

    //It has no neighbours, we must still add it to the prm though
    if(node.second.size() == 0){
      prm.push_back(std::make_pair(pCurrent, pCurrent));
    }

    for(auto const &neighbour: node.second){
      cv::Point pNeighbour = lmap_.convertToPoint(reference_, network_[neighbour.first]);

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

std::vector<cv::Point> GlobalMap::toPointPath(std::vector<TGlobalOrd> path){
  std::vector<cv::Point> pointPath;
  for(auto const &ord: path){
    pointPath.push_back(lmap_.convertToPoint(reference_, ord));
  }

  return pointPath;
}

std::vector<TGlobalOrd> GlobalMap::toOrdPath(std::vector<vertex> path){
  std::vector<TGlobalOrd> ordPath;

  for(auto const &v: path){
    ordPath.push_back(network_[v]);
  }

  return ordPath;
}

/*! @brief Calculates the euclidean distance between two ordiantes.
 *
 *  @param p1 The first point.
 *  @param p2 The second point
 *  @return double - The distance between the two points.
 */
double distance(TGlobalOrd p1, TGlobalOrd p2){
  double a = std::abs(p2.x - p1.x);
  double b = std::abs(p2.y - p1.y);

  return std::sqrt(std::pow(a, 2) + std::pow(b, 2));
}

void GlobalMap::connectNode(cv::Mat &cspace, vertex node, bool imposeMaxDist){
  int cnt(0);
  TGlobalOrd nodeOrd = network_[node];

  //Find the neighbours that are closest to the current node
  std::vector<vertex> candidates;
  for(auto const &candidate: network_){
    if(graph_.canConnect(candidate.first)){
      //If we care about the distance, then we need to check before adding.
      if(imposeMaxDist){
        if(distance(nodeOrd, candidate.second) < MaxDistance){
          candidates.push_back(candidate.first);
        }
      } else {
        candidates.push_back(candidate.first);
      }
    }
  }

  //TODO: Sort candidates by distance.

  //For each of our neighbouring candidates, determine
  //if we can connect to them in the cspace
  for(auto const &neighbour: candidates){
    TGlobalOrd ordN = network_[neighbour];

    cv::Point pCurrent = lmap_.convertToPoint(reference_, nodeOrd);
    cv::Point pN = lmap_.convertToPoint(reference_, ordN);

    if(lmap_.canConnect(cspace,pCurrent,pN)){
      if(graph_.addEdge(node, neighbour, distance(nodeOrd, ordN))){
      cnt++;
      }
    }

    if(cnt > MaxGraphDensity){
      break; //The current node has reached its max capacity
    }
  }
}

void GlobalMap::connectNodes(cv::Mat &m, bool imposeMaxDist){
  for(auto const &entry: network_){
    connectNode(m, entry.first, imposeMaxDist);
  }
}

vertex GlobalMap::findOrAdd(TGlobalOrd ordinate){
  vertex v;
  if(existsAsVertex(ordinate)){
    lookup(ordinate, v);
  } else {
    //Generate a new vertex and add to graph, also adding
    //to the internal lookup table.
    v = nextVertexId();
    graph_.addVertex(v);
    network_.insert(std::make_pair(v, ordinate));
  }

  return v;
}

bool GlobalMap::existsAsVertex(TGlobalOrd ord){
  for(auto &v: network_){
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
  for(auto &vert: network_){
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
