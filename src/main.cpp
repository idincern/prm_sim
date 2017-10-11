#include <cstdio>
#include <iostream>
#include <cstdlib>

//#include "graph.h"
#include "house.h"

int main() {
  House h;

  //Build floor plan with rooms
  h.addVertex("kitchen");
  h.addVertex("master");
  h.addVertex("kids");
  h.addVertex("hallway");
  h.addVertex("toilet");
  h.addVertex("living");

  //Add doorways
  h.addEdge("kitchen", "toilet");
  h.addEdge("kitchen", "hallway");
  h.addEdge("master", "hallway");
  h.addEdge("kids", "hallway");
  h.addEdge("living", "hallway");
  h.addEdge("toilet", "living");

  //h.printGraph();


  std::vector<vertex> path = h.shortestPath("kids", "toilet");

  for(auto room: path){
    std::cout << room << " -> ";
  }

  std::cout << std::endl;
}



