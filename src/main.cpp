#include <cstdio>
#include <iostream>
#include <cstdlib>

#include "graph.h"

int main() {
  Graph g(7);

  g.addVertex(0);
  g.addVertex(1);
  g.addVertex(2);
  g.addVertex(3);
  g.addVertex(4);
  g.addVertex(5);
  g.addVertex(6);
  g.addVertex(7);
  g.addVertex(8);

  g.addEdge(0, 1, 3.0);
  g.addEdge(0, 2, 7.0);
  g.addEdge(0, 3, 5.0);

  g.addEdge(1, 4, 7.0);
  g.addEdge(1, 2, 1.0);

  g.addEdge(2, 3, 3.0);
  g.addEdge(2, 4, 2.0);
  g.addEdge(2, 5, 1.0);
  g.addEdge(2, 6, 3.0);

  //g.addEdge(3, 6, 2.0);

  g.addEdge(4, 5, 2.0);
  g.addEdge(4, 7, 1.0);

  g.addEdge(5, 7, 3.0);
  g.addEdge(5, 6, 3.0);
  //g.addEdge(5, 8, 2.0);

  g.addEdge(6, 8, 4.0);
  g.addEdge(7, 8, 5.0);

  //g.printGraph();


  std::vector<vertex> path = g.shortestPath(0, 8);

  for(vertex v: path){
    std::cout << v << " -> ";
  }

  std::cout << std::endl;
}



