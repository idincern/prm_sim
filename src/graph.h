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
#ifndef GRAPH_H
#define GRAPH_H

#include <set>
#include <map>
#include <utility>
#include <vector>

typedef unsigned int vertex;            /*!< A vertex has a unique id within the adjacency list */
typedef double weight;                  /*!< An edge weighting is non-negative */
typedef std::pair<vertex, weight> edge; /*!< An edge points to a vertex and has a weighting */
typedef std::set<edge> edges;           /*!< A list of edges (or neighbours) */

class Graph
{
public:
  /*! @brief Constructor for Graph.
   *
   *  @param maxNeighbours The max amount of neighbours a vertex can have.
   */
  Graph(unsigned int maxNeighbours, weight maxWeight);

  /*! @brief Adds a vertex to the graph.
   *
   *  @param v The unique vertex to add to the graph.
   *  @return bool - Will return false if vertex already exists within graph.
   */
  bool addVertex(const vertex v);

  /*! @brief Adds a weighted edge between two verticies.
   *
   *  @param v The first vertex.
   *  @param u The second vertex.
   *  @param w The weight of the edge.
   *  @return bool - Will return false if either vertex does not exist or
   *                 either vertex has exceeded its max amount of neighbours.
   */
  bool addEdge(const vertex v, const vertex u, const weight w);

  /*! @brief Finds the shortest path between two verticies in the graph.
   *
   *  This function uses Dijkstra's algorithm for finding the shortest
   *  path between two verticies in a weighted graph.
   *
   *  @param start The start vertex.
   *  @param goal The end vertex, the goal to reach.
   *  @return vector - Will return a vector of verticies that represent
   *                   the shortest path between start and goal. This
   *                   vector will be empty if there is no path.
   */
  std::vector<vertex> shortestPath(const vertex start, const vertex goal);

  /*! @brief Returns the container representing the graph.
   *
   *  @return map<vertex, edges> - The container that represents the graph.
   */
  std::map<vertex, edges> container() const;

  /*! @brief Checks if one is able to connect to a given vertex.
   *
   *  This is determined by the supplied weighting, and whether
   *  the vertex has connections to spare.
   *
   *  @param v The vertex to check for.
   *  @param w The potential weighting.
   *  @return TRUE - If we can connect
   */
  bool canConnect(const vertex v, const weight w);

private:
  weight maxWeight_;                   /*!< The max weighting an edge is allowed to have */
  unsigned int maxNeighbours_;         /*!< A vertex has a max amount of neighbours */
  std::map<vertex, edges> container_;  /*!< A container of all verticies and their neighbours (edges) */

  /*! @brief Constructs the path between start and goal.
   *
   *  @param parents Used to determine the parents of each node all the way back to the start.
   *  @param goal The end vertex, the goal to reach.
   *  @return vector - Will return a vector of verticies that represent
   *                   the shortest path between start and goal. This
   *                   vector will be empty if there is no path.
   */
  std::vector<vertex> constructPath(std::map<vertex, vertex> parents, vertex goal);
};

#endif // GRAPH_H
