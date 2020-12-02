#include "Graph.h"
#include <iostream>
#include <list>
using std::list;


int main()
{
    //Graph g("data/airports.dat", "data/routes.dat");
    //g.Dijkstra(1,746);
    
    // Following Code Test Dijkstra Algorithm based on a small hand-drawn graph.
    // You can draw it on paper to verify.
    Graph g;
    g.insertVertex(Graph::Vertex(1, "", "", "", "", "", 0, 0));
    g.insertVertex(Graph::Vertex(2, "", "", "", "", "", 0, 0));
    g.insertVertex(Graph::Vertex(3, "", "", "", "", "", 0, 0));
    g.insertVertex(Graph::Vertex(4, "", "", "", "", "", 0, 0));
    g.insertVertex(Graph::Vertex(5, "", "", "", "", "", 0, 0));
    g.insertVertex(Graph::Vertex(6, "", "", "", "", "", 0, 0));
    g.insertEdge(1,2,3);
    g.insertEdge(2,3,1);
    g.insertEdge(1,3,5);
    g.insertEdge(1,5,5);
    g.insertEdge(2,4,2);
    g.insertEdge(3,4,1);
    g.insertEdge(3,5,1);
    g.insertEdge(4,5,3);
    g.insertEdge(4,6,2);
    g.insertEdge(5,6,4);
    g.printVertex();
    g.printEdge();
    g.Dijkstra(1,6);
    g.Dijkstra(1,2);
    g.Dijkstra(3,6);
    g.Dijkstra(2,5);
    // Correctly returns path of length 0 with 1 vertex on the path.
    g.Dijkstra(1,1);
    g.Dijkstra(6,6);
    return 0;
}