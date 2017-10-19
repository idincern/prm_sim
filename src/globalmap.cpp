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
static const double MaxDistance = 1.5;          /*!< The max distance between two verticies in the graph (2.5) */
static const double DefaultMapSize = 20.0;      /*!< The default ogmap size is 20x20m */
static const double DefaultfResolution = 0.1;   /*!< The default ogmap resolution is 0.1 */

GlobalMap::GlobalMap():
  graph_(Graph(MaxGraphDensity)), lmap_(LocalMap(DefaultMapSize, DefaultfResolution))
{
  mapSize_ = DefaultMapSize;   //TODO: This should be queried from local map
  nextVertexId_ = 0;
  reference_.x = 0;
  reference_.y = 0;
}

GlobalMap::GlobalMap(double mapSize, double mapRes):
  graph_(Graph(MaxGraphDensity)), lmap_(LocalMap(mapSize, mapRes))
{
  mapSize_ = mapSize;
  nextVertexId_ = 0;
  reference_.x = 0;
  reference_.y = 0;
}

std::vector<TGlobalOrd> GlobalMap::build(cv::Mat &cspace, TGlobalOrd start, TGlobalOrd goal)
{
  vertex vStart, vGoal;     //Vertex representation of the global ords
  std::vector<TGlobalOrd> path;

  //Check that both ordinates are accessible
  if(!ordinateAccessible(cspace, start) || !ordinateAccessible(cspace, goal)){
    return path;
  }

  //If both in the graph, perhaps there is already a path?
  if(existsAsVertex(start) && existsAsVertex(goal))
  {
    lookup(start, vStart);
    lookup(start, vGoal);

    std::vector<vertex> vPath = graph_.shortestPath(vStart, vGoal);
    if(vPath.size() > 0){
      return optimisePath(cspace, toOrdPath(vPath));
    }
  }

  unsigned int attempts(0);
  while(attempts < (mapSize_ * mapSize_)){
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
    if(!lmap_.isAccessible(cspace, pRand)){
      continue;
    }

    //See if vertex exists in graph, if not - add!
    findOrAdd(randomOrd);
    attempts++;
  }

  //Connect all the newely generated nodes (verticies)
  connectNodes(cspace, true);

  //Find or add the start/goal verticies to/in our graph
  vStart = findOrAdd(start);
  vGoal = findOrAdd(goal);

  //Connect to the PRM
  connectNode(cspace, vStart, false);
  connectNode(cspace, vGoal, false);

  //Find a path and optimise the graph.
  std::vector<vertex> vPath = graph_.shortestPath(vStart, vGoal);

  if(vPath.size() > 0){
    return optimisePath(cspace, toOrdPath(vPath));
  }

  return path;
}

std::vector<TGlobalOrd> GlobalMap::query(cv::Mat &cspace, TGlobalOrd start, TGlobalOrd goal){
  //This function connects the start and end goal to the graph
  //and attempts to find a connection through the prm
  vertex vStart = findOrAdd(start);
  vertex vGoal = findOrAdd(goal);

  connectNode(cspace, vStart, true);
  connectNode(cspace, vStart, true);

  //Find a path and optimise the graph.
  std::vector<vertex> vPath = graph_.shortestPath(vStart, vGoal);
  if(vPath.size() > 0){
    return optimisePath(cspace, toOrdPath(vPath));
  }

  //TODO: THIS IS WIP
  return std::vector<TGlobalOrd>();
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
  TGlobalOrd nodeOrd = network_[node];

  if(!graph_.canConnect(node)){
    //No point trying to connect a node that can't
    //be connected to.
    return;
  }

  //Find the neighbours that are closest to the current node
  std::vector<TGlobalOrd> candidates;
  for(auto const &candidate: network_){
    if(graph_.canConnect(candidate.first)){
      //If we care about the distance, then we need to check before adding.
      if(imposeMaxDist){
        if(distance(nodeOrd, candidate.second) < MaxDistance){
          candidates.push_back(candidate.second);
        }
      } else {
        candidates.push_back(candidate.second);
      }
    }
  }

  //Sort candidates by distance.
  std::sort(candidates.begin(), candidates.end(),[nodeOrd](const TGlobalOrd &lhs, const TGlobalOrd &rhs){
    return distance(lhs, nodeOrd) < distance(rhs, nodeOrd);});

  //For each of our neighbouring candidates, determine
  //if we can connect to them in the cspace
  for(auto const &neighbour: candidates){
    vertex vNeighbour;
    if(!lookup(neighbour, vNeighbour)){
      //something went wrong adding this neighbour, continue
      continue;
    }

    cv::Point pCurrent = lmap_.convertToPoint(reference_, nodeOrd);
    cv::Point pN = lmap_.convertToPoint(reference_, neighbour);
    if(lmap_.canConnect(cspace,pCurrent,pN)){
      graph_.addEdge(node, vNeighbour, distance(nodeOrd, neighbour));
    }

    if(!graph_.canConnect(node)){
      break; //We've reached the capacity for this node
    }
  }
}

unsigned int GlobalMap::neighboursInDist(cv::Mat &cspace, TGlobalOrd nodeOrd, double dist){
  unsigned int candidates(0);

  for(auto const &candidate: network_){
    //Can we connect to candidate
    if(graph_.canConnect(candidate.first)){
      //Is candidate within some heuristic pre-defined distance
      if(distance(nodeOrd, candidate.second) < dist){
        //Can we physically connect to the candidate
        cv::Point pCurrent = lmap_.convertToPoint(reference_, nodeOrd);
        cv::Point pN = lmap_.convertToPoint(reference_, candidate.second);

        if(lmap_.canConnect(cspace,pCurrent,pN)){
          candidates++;
        }
      }
    }
  }

  return candidates;
}


void GlobalMap::connectNodes(cv::Mat &cspace, bool imposeMaxDist){
  //Prioritise nodes by number of neighbours in vicinity
  std::vector<std::pair<vertex, unsigned int>> nodeOrder;
  for(auto const &entry: network_){
    if(graph_.canConnect(entry.first)){
      nodeOrder.push_back(
            std::make_pair(entry.first, neighboursInDist(cspace, entry.second, MaxDistance)));
    }
  }

  //Prioritise connection order by nodes who have the least amount of available connections
  //within some pre-determined distance
  std::sort(nodeOrder.begin(), nodeOrder.end(),
            [](const std::pair<vertex, unsigned int> &lhs, std::pair<vertex, unsigned int> &rhs){
    return lhs.second < rhs.second;
  });

  for(auto const n: nodeOrder){
    connectNode(cspace, n.first, imposeMaxDist);
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
      return true; //TODO: USE STRUCT == instead?
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

void GlobalMap::setResolution(double resolution)
{
  lmap_.setResolution(resolution);
}

bool GlobalMap::ordinateAccessible(cv::Mat &cspace, TGlobalOrd ordinate)
{
  return existsAsVertex(ordinate) ||
      lmap_.isAccessible(cspace, lmap_.convertToPoint(reference_, ordinate));
}

void GlobalMap::expandConfigSpace(cv::Mat &space, double robotDiameter)
{
  lmap_.expandConfigSpace(space, robotDiameter);
}

