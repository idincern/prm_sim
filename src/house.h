#ifndef HOUSE_H
#define HOUSE_H

#include "graph.h"
#include <vector>

class House: public Graph
{
public:
    House();
    std::vector<vertex> shortestPath(const vertex roomStart, const vertex roomEnd);
};

#endif // HOUSE_H
