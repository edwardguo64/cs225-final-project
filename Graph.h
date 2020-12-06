#pragma once

#include <string>
using std::string;
#include <list>
using std::list;
#include <map>
using std::map;
#include <iostream>
using std::cout;
using std::cin;
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
#include <limits>

class Graph
{
    public:
        enum v_label {UNEXPLORED, VISITED};
        enum e_label {UNDISCOVERED, DISCOVERY, CROSS};
        

        // The Vertex struct contains all of the fundamental information regarding each airport.
        // Each airport has a unique airportID_ that is always defined.
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

            Vertex()
            {

            }

            bool operator>(const Vertex & other) const
            {
                return airportID_ > other.airportID_;
            }

            bool operator<(const Vertex & other) const
            {
                return airportID_ < other.airportID_;
            }

            bool operator==(const Vertex & other) const {
                return airportID_ == other.airportID_;
            }

            bool operator!=(const Vertex & other) const {
                return airportID_ != other.airportID_;
            }
        }Vertex;
        
        /* The Edge struct is analagous to an airport route, which displays the relationship between 
         * two airport Vertices
         */
        typedef struct Edge
        {

            const Vertex * firstID_;
            const Vertex * secondID_;
            e_label label_;
            double weight_; // defined as physical distance between the two connected airports

            Edge(const Vertex * firstID, const Vertex * secondID, double weight = 0):
            firstID_(firstID), secondID_(secondID), weight_(weight)
            {

            }

        }Edge;

        
        // Maps an airportID_ to its specific airport Vertex.
        map<int, Vertex> converter_;   // ID_to_vertex_;             
        // Maps an airport Vertex to its degree (the number of airports it is connected to by an edge).
        map<Vertex, int> degree_map_;      // vertex_to_deg_;         
        // Maps an airport Vertex to a list of edge pointers (relationship with other airports).
        map<Vertex, list<Edge *>> adjacency_list_;  
        // List of all related airports
        list<Edge> edge_list_;      
        map<Vertex, v_label> vertex_labels_;    // vertex_to_labels_;
        // Maps airport ICAO codes to airport ID
        map<string, int> ICAO_map_;

    public:
        // Graph constructor with airport and route data filenames passed in for parsing.
        Graph(const string & airport, const string & route);

        Graph();

        // Insert a vertex into the graph.
        void insertVertex(Vertex v);

        // Insert an edge into the graph.
        // Argument first and second are airportID_.
        void insertEdge(int first, int second, double weight);

        // Returns a list of all related airports to the given airport Vertex.
        list<Edge *> incidentEdges(const Vertex & v);
        Edge* areAdjacent(const Vertex & v1, const Vertex & v2);
        
        void parseAirport(const string & filename);
        void parseRoute(const string & filename);

        void printVertex();
        void printEdge();
        void printICAOmap();

        // Helper function which returns the physical distance between two airports given their exact locations.
        double distance(double lat1, double long1, double lat2, double long2);

        double getWeight(const Edge & e);

        vector<Vertex> getVertices();
        vector<Edge *> getEdges();
        
        vector<Vertex> getAdjacent(const Vertex & v);

        v_label getVLabel(const Vertex & v);
        void setVLabel(Vertex & v, v_label label);

        e_label getELabel(const Edge *e);
        void setELabel(Edge *e, e_label label);

        void insertICAO(string ICAO, int id);

        int getID(string ICAO);

        void BFS();
        void BFS(Vertex & v);

        void DFS();
        void DFS(Vertex & v);

        list<Vertex> Dijkstra(int sourceID, int destinationID, double * distance);
        list<Vertex> landmark(int sourceID, int stopID, int destID, double * distance);

		Vertex ID_to_Vertex(int portID);

        /* Exists for displaying purposes when vertices are private */
        int Vertex_to_ID(Vertex v);
};