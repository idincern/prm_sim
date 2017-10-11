#include "house.h"
#include <set>
#include <queue>
#include <iostream>
#include <algorithm>

House::House()
{
}

std::vector<vertex> reconstruct_path(std::map<vertex, vertex> parents, vertex goal){
  std::vector<vertex> path;
  path.push_back(goal);

  while(parents.find(path.back()) != parents.end()){
    path.push_back(parents.at(path.back()));
  }

  std::reverse(path.begin(), path.end());
  return path;
}


std::vector<vertex> House::shortestPath(const vertex roomStart, const vertex roomEnd)
{
    //Breadth First Search
    std::map<vertex, vertex> parents; //used to reconstruct the shortest path
    std::queue<vertex> q;
    std::set<vertex> visited;

    visited.insert(roomStart);
    q.push(roomStart);

    while(!q.empty()){
        vertex current = q.front();
        q.pop();

        if(current == roomEnd){
            return reconstruct_path(parents, current);
        }

        //Get neighbours
        edges neighbours = container.find(current)->second;

        //for each neighbour, visit it
        for(auto neighbour: neighbours){
            if(visited.find(neighbour) == visited.end()){
                q.push(neighbour);
                visited.insert(neighbour);

                parents[neighbour] = current;
            }
        }
    }
}
