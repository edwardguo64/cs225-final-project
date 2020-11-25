#pragma once

#include <string>
using std::string;
#include <list>
using std::list;
#include <map>
using std::map;
#include <iostream>
#include <fstream>
using std::ifstream;
#include <sstream>
#include <vector>
using std::vector;
#include <cmath>

class Graph
{
    private:

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

        }Vertex;
        

        typedef struct Edge
        {

            const Vertex * firstID_;
            const Vertex * secondID_;
            double weight_;

            Edge(const Vertex * firstID, const Vertex * secondID, double weight = 0):
            firstID_(firstID), secondID_(secondID), weight_(weight)
            {

            }

        }Edge;

        map<int, Vertex> converter_;
        map<Vertex, int> degree_map_;
        map<Vertex, list<Edge *>> adjacency_list_;
        list<Edge> edge_list_;

    public:

        Graph(const string & airport, const string & route);
        void insertVertex(Vertex v);
        void insertEdge(int first, int second, double weight);
        list<Edge *> incidentEdges(const Vertex & v);
        bool areAdjacent(const Vertex & v1, const Vertex & v2);
        
        void parseAirport(const string & filename);
        void parseRoute(const string & filename);

        void printVertex();
        void printEdge();

        double distance(double lat1, double long1, double lat2, double long2);

};