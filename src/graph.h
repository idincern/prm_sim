//A simple undirected, adjacency list
#ifndef GRAPH_H
#define GRAPH_H

#include <set>
#include <map>
#include <utility>
#include <vector>

typedef unsigned int vertex;            /*!< A vertex has a unique id within the adjacency list */
typedef double weight;                  /*!< An edge weighting is non-negative */
typedef std::pair<vertex, weight> edge; /*!< An edge points to a vertex and has a weighting */
typedef std::set<edge> edges;           /*!< A list of edges/neighbours */

class Graph
{
public:
  Graph(unsigned int maxNeighbours);

  bool addVertex(const vertex v);                                 //Add vertex to the graph
  bool addEdge(const vertex v, const vertex u, const weight w);   //Add edge between two verticies with weight

  std::vector<vertex> shortestPath(const vertex start, const vertex goal);

  //TODO: Need??
  void printGraph();
private:
  unsigned int maxNeighbours_;         /*!< A vertex has a max amount of neighbours */
  std::map<vertex, edges> container_;  /*!< A container of all verticies and their neighbours (edges) */

  std::vector<vertex> constructPath(std::map<vertex, double> distances, const vertex start, const vertex goal);
};

#endif // GRAPH_H
