//A simple undirected, adjacency list
#include "graph.h"
#include <iostream>

Graph::Graph()
{
}

bool Graph::addVertex(const vertex v)
{
  if (container.find(v) != container.end()){
    return false; //the vertex already exists witin the graph
  }

  //inset a new vertex with edges
  container.insert(std::pair<vertex, edges>(v, std::set<vertex>()));

  return true;
}

bool Graph::addEdge(const vertex v, const vertex u)
{
  if(container.find(v) == container.end() || container.find(u) == container.end()){
    return false; //All verticies must be present when adding an edge
  }

  container.find(v)->second.insert(u);
  container.find(u)->second.insert(v);

  return true;
}

void Graph::printGraph(){
  for(auto const &c: container){
    std::cout << c.first << " -> ";

    for(auto const &e: c.second){
      std::cout << e << ", ";
    }

    std::cout << std::endl;
  }
}

