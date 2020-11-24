#include "Graph.h"
#include <iostream>



int main()
{
    Graph g("test_airports.dat", "test_routes.dat");
    g.printVertex();
    g.printEdge();
    return 0;
}