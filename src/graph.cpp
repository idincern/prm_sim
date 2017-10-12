/*! @file
 *
 *  @brief A weighted undirected adjacency list.
 *
 *  This class contains the methods for defining a weighted undirected
 *  adjacency list (a Graph). It also contains a function to find the
 *  shortest path between two verticies.
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#include "graph.h"

#include <algorithm>
#include <limits>
#include <iostream>
#include <string>

Graph::Graph(unsigned int maxNeighbours): maxNeighbours_(maxNeighbours)
{
}

bool Graph::addVertex(const vertex v)
{
  if (container_.find(v) != container_.end()){
    return false; //the vertex already exists witin the graph
  }

  //inset a new vertex with no neighbours
  container_.insert(std::pair<vertex, edges>(v, edges()));

  return true;
}

bool Graph::addEdge(const vertex v, const vertex u, const weight w)
{
  if(container_.find(v) == container_.end() || container_.find(u) == container_.end()){
    return false; //All verticies must be present when adding an edge
  }

  //TODO: Determine correct behaviour
  //Check the verticies are not at their neighbour limits
  if(container_.find(v)->second.size() >= maxNeighbours_ ||
     container_.find(u)->second.size() >= maxNeighbours_){
    return false;
  }

  container_.find(v)->second.insert(edge(u, w));
  container_.find(u)->second.insert(edge(v, w));

  return true;
}

/*! @brief Finds the closest vertex in the queue.
 *
 *  @param distances This contains the distance between each vertex and the start.
 *  @param q The queue of available verticies to search within.
 *  @return vertex - The next closest vertex.
 *
 *  @note This assumes that q is non-empty.
 */
vertex closestVertex(std::map<vertex, double> distances, std::vector<vertex> q){
  vertex minVertex = q.at(0);
  double minDist = std::numeric_limits<double>::infinity();

  for(auto const &v: q){
    if(distances[v] < minDist){
      minVertex = v;
      minDist = distances[v];
    }
  }

  return minVertex;
}

std::vector<vertex> Graph::constructPath(std::map<vertex, double> distances, const vertex start, const vertex goal){
  std::vector<vertex> path;

  if(distances[goal] == std::numeric_limits<double>::infinity()){
    return path; //The goal has not been reached, return an empty path
  }

  path.push_back(goal);
  vertex current = goal;

  //While the start vertex has not been connected to the goal
  while(current != start){
    double minD = std::numeric_limits<double>::infinity();
    vertex minV = current;

    edges neighbours = container_[current];
    for(auto const &n: neighbours){
      double totalD = distances[n.first] + n.second;
      if(totalD < minD){
        minD = totalD;
        minV = n.first;
      }
    }

    path.push_back(minV);
    current = minV;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<vertex> Graph::shortestPath(const vertex start, const vertex goal){
  //This map contains the distances between various nodes and the start node.
  std::map<vertex, double> distances;
  std::vector<vertex> queue, path;

  if(container_.find(start) == container_.end() ||
     container_.find(goal) == container_.end()){
    return path; //Empty path between two unknown verticies
  }

  for(auto const &v: container_){
    //Initially, distances to other verticies is equal to infinity
    distances.insert(std::pair<vertex, double>(v.first, std::numeric_limits<double>::infinity()));
    queue.push_back(v.first);
  }

  //For the start position the distance to itself is 0
  distances[start] = 0;

  while(!queue.empty())
  {
    vertex v = closestVertex(distances, queue);
    edges neighbours = container_[v];

    for(auto const &n: neighbours)
    {
      double alt = distances[v] + n.second; //neighbour distance + weight
      if(alt < distances[n.first]){
        distances[n.first] = alt;
      }
    }

    queue.erase(std::remove(queue.begin(), queue.end(), v), queue.end());
  }

  return constructPath(distances, start, goal);
}
