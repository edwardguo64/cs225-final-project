#pragma once

#include <string>
using std::string;
#include <list>
using std::list;
#include <map>
using std::map;
#include <iostream>
using std::cout;
using std::endl;
#include <fstream>
using std::ifstream;
#include <sstream>
#include <vector>
using std::vector;
#include <cmath>
#include <queue>
using std::queue;
#include <stack>
using std::stack;

class Graph
{
    private:
        enum v_label {UNEXPLORED, VISITED};
        enum e_label {UNDISCOVERED, DISCOVERY, CROSS};

        //the Vertex struct contains all of the fundamental information regarding each airport
        typedef struct Vertex
        {
            int airportID_;
            string name_;
            string city_;
            string country_;
            string IATA_;
            string ICAO_;
            double latitude_;
            double longitude_;

            Vertex(int airportID, string name, string city, string country, string IATA, string ICAO, double latitude, double longitude):
            airportID_(airportID), name_(name), city_(city), country_(country), IATA_(IATA), ICAO_(ICAO), latitude_(latitude), longitude_(longitude) 
            {

            }

            bool operator<(const Vertex & other) const
            {
                return airportID_ < other.airportID_;
            }

            bool operator==(const Vertex & other) const {
                return airportID_ == other.airportID_;
            }

        }Vertex;
        
        /*the Edge struct is analagous to an airport route, which displays the relationship between 
        two airport Vertices
        */
        typedef struct Edge
        {

            const Vertex * firstID_;
            const Vertex * secondID_;
            e_label label_;
            double weight_; //distance between the two related airports

            Edge(const Vertex * firstID, const Vertex * secondID, double weight = 0):
            firstID_(firstID), secondID_(secondID), weight_(weight)
            {

            }

        }Edge;

        
        //maps an airportID # to its specific airport Vertex
        map<int, Vertex> converter_;                
        //maps a airport Vertex to its degree (however many other airports it is connected to)
        map<Vertex, int> degree_map_;               
        //maps an airport Vertex to a list of edge pointers (relationship with other airports)
        map<Vertex, list<Edge *>> adjacency_list_;  
        //list of all related airports
        list<Edge> edge_list_;      
        map<Vertex, v_label> vertex_labels_;                

    public:
        //graph constructor with airport and route data files passed in for parsing
        Graph(const string & airport, const string & route);
        void insertVertex(Vertex v);
        void insertEdge(int first, int second, double weight);
        //returns a list of all related airports to the given airport Vertex
        list<Edge *> incidentEdges(const Vertex & v);
        Edge* areAdjacent(const Vertex & v1, const Vertex & v2);
        
        void parseAirport(const string & filename);
        void parseRoute(const string & filename);

        void printVertex();
        void printEdge();

        //helper function which returns the distance between two airports given their exact locations
        double distance(double lat1, double long1, double lat2, double long2);

        double getWeight(const Edge & e);

        vector<Vertex> getVertices();
        vector<Edge *> getEdges();
        vector<Vertex> getAdjacent(const Vertex & v);

        v_label getVLabel(const Vertex & v);
        void setVLabel(Vertex & v, v_label label);

        e_label getELabel(const Edge *e);
        void setELabel(Edge *e, e_label label);

        void BFS();
        void BFS(Vertex & v);

        void DFS();
        void DFS(Vertex & v);
};