#include "Graph.h"
#include <iostream>



int main()
{
    Graph g("data/airports.dat", "data/routes.dat");
    g.printVertex();
    g.printEdge();
    return 0;
}