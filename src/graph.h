//A simple undirected, adjacency list
#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <set>
#include <map>

typedef std::string vertex;     //The only information a vertex will hold is its own name
typedef std::set<vertex> edges; //An edge is simply a pointer to another vertex

class Graph
{
public:
  Graph();

  bool addVertex(const vertex v);                 //Add vertex to the graph
  bool addEdge(const vertex v, const vertex u); //Add edge between two verticies

  void printGraph();
protected:
  std::map<vertex, edges> container; //Contains all the links between vertexes etc.
};

#endif // GRAPH_H
