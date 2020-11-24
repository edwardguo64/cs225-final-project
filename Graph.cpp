#include "Graph.h"

Graph::Graph(const string & airport, const string & route)
{
    parseAirport(airport);
    parseRoute(route);
}

void Graph::parseAirport(const string & filename)
{

    ifstream file(filename);
    string data = "";
    int counter = 0;

    int airportID;
    string name;
    string city;
    string country;
    string IATA;
    string ICAO;
    double latitude;
    double longitude;
    
    if(file.is_open())
    {
        while(getline(file, data,','))
        {
            if(counter == 0)
            {
                airportID = std::stoi(data);
            }
            else if(counter == 1)
            {
                name = data;
            }
            else if(counter == 2)
            {
                city = data;
            }
            else if(counter == 3)
            {
                country = data;
            }
            else if(counter == 4)
            {
                IATA = data;
            }
            else if(counter == 5)
            {
                ICAO = data;
            }
            else if(counter == 6)
            {
                latitude = std::stod(data);
            }
            else if(counter == 7)
            {
                longitude = std::stod(data);
                insertVertex(Vertex(airportID, name, city, country, IATA, ICAO, latitude, longitude, 0));
                // Move onto next line.
                getline(file, data);
                counter = 0;
                continue;
            }
            counter++;

        }
        file.close();
    }

}

void Graph::parseRoute(const string & filename)
{
    ifstream file(filename);
    string data = "";
    int counter = 0;
    int firstID;
    int secondID;

    if(file.is_open())
    {
        while(getline(file, data,','))
        {
            if(counter == 3)
            {
                if(data == "\\N")
                {
                    getline(file, data);
                    counter = 0;
                    continue;
                }
                else
                {
                    firstID = std::stoi(data);
                }
            }
            else if(counter == 5)
            {
                if(data == "\\N")
                {
                    getline(file, data);
                    counter = 0;
                    continue;
                }
                else
                {
                    secondID = std::stoi(data);
                }
                insertEdge(firstID, secondID, 0);

                getline(file, data);
                counter = 0;
                continue;
            }
            counter++;

        }
        file.close();
    }

}


void Graph::insertVertex(Vertex v)
{
    
    std::pair<Vertex, list<Edge *>> ret(v, list<Edge *>());
    adjacency_list_.insert(ret);
    std::pair<int, Vertex> ret2(v.airportID_, v);
    converter_.insert(ret2);
}

// Needs to check if either (u, v) or (v, u) is in the graph already.
void Graph::insertEdge(int first, int second, double weight)
{
    if(converter_.find(first) == converter_.end())
    {
       return;
    }

    if(converter_.find(second) == converter_.end())
    {
        return;
    }

    if(areAdjacent(converter_.find(first)->second, converter_.find(second)->second))
    {
        return;
    }
    Edge e(&(converter_.find(first)->second), &(converter_.find(second)->second), weight);
    

    // Compute weight.

    edge_list_.push_back(e);
}

bool Graph::areAdjacent(const Vertex & v1, const Vertex & v2)
{
    return false;
}

list<Graph::Edge *> Graph::incidentEdges(const Vertex & v)
{
    return list<Edge *>();
}

void Graph::printVertex()
{
    for(auto it = adjacency_list_.begin(); it != adjacency_list_.end(); it++)
    {
        std::cout << it->first.airportID_ << " is adjacent to ";
        
        for(auto it2 = it->second.begin(); it2 != it->second.end(); it2++)
        {
            std::cout << "(" << (*it2)->firstID_->airportID_ << ", " << (*it2)->secondID_->airportID_ <<  ") " << std::endl;
        }
    }
}

void Graph::printEdge()
{
    for(auto it = edge_list_.begin(); it != edge_list_.end(); it++)
    {
        std::cout << it->firstID_->airportID_ << " connected to " << it->secondID_->airportID_ << std::endl;
    }
}