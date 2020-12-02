#include "Graph.h"

Graph::Graph(const string & airport, const string & route)
{
    parseAirport(airport);
    parseRoute(route);
}

Graph::Graph()
{

}

/* Pulled from: https://www.geeksforgeeks.org/program-distance-two-points-earth/ 
 * Returns distance in km
 */
double Graph::distance(double lat1, double long1, double lat2, double long2) {
    double lat1rad = lat1 / 180 * M_PI;
    double long1rad = long1 / 180 * M_PI;
    
    double lat2rad = lat2 / 180 * M_PI;
    double long2rad = long2 / 180 * M_PI;

    double longdif = long2rad - long1rad;

    return 3963.0 * acos((sin(lat1rad) * sin(lat2rad)) + cos(lat1rad) * cos(lat2rad) * cos(longdif)) * 1.609344;
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
        while(getline(file, data,',')){     //stores comma-separated strings in data till end of file
            if(counter == 0){
                airportID = std::stoi(data);
            }
            else if(counter == 1){
                name = data;
                // Check if the name has a comma in it
                while (data[data.size() - 1] != '\"') { //until the last char of data is "
                    getline(file, data, ',');           //read and store next piece of comma-separated string 
                    name += "," + data;                 //concatenate to include entire string between " "
                }
            }
            else if(counter == 2){
                city = data;
                // Check if the city has a comma in it
                while (data[data.size() - 1] != '\"') {
                    getline(file, data, ',');
                    name += "," + data;
                }
            }
            else if(counter == 3){
                country = data;
                // Check if the country has a comma in it
                while (data[data.size() - 1] != '\"') {
                    getline(file, data, ',');
                    name += "," + data;
                }
            }
            else if(counter == 4){
                IATA = data;
            }
            else if(counter == 5){
                ICAO = data;
            }
            else if(counter == 6){
                latitude = std::stod(data);
            }
            else if(counter == 7){
                longitude = std::stod(data);
                insertVertex(Vertex(airportID, name, city, country, IATA, ICAO, latitude, longitude));
                // Move on to next line to disregard irrelevant data.
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

    if(file.is_open()){
        while(getline(file, data,',')){ //stores comma-separated strings in data till end of file
            if(counter == 3){           //stores fourth entry (route's source airport) of each line
                if(data == "\\N"){      //if invalid entry, move on to next line
                    getline(file, data);
                    counter = 0;
                    continue;
                }
                else{
                    firstID = std::stoi(data);
                }
            }
            else if(counter == 5){      //stores sixth entry (route's destination airport) of each line
                if(data == "\\N"){      //if invalid entry, move on to next line
                    getline(file, data);
                    counter = 0;
                    continue;
                }
                else{
                    secondID = std::stoi(data);
                }
                //converts airport ID int to identify specific airport vertex
                Vertex first_vertex = converter_.find(firstID)->second;
                Vertex second_vertex = converter_.find(secondID)->second;
                insertEdge(firstID, secondID, distance(first_vertex.latitude_, first_vertex.longitude_, 
                                                        second_vertex.latitude_, second_vertex.longitude_));

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
    //if first airport does not exist disregard
    if(converter_.find(first) == converter_.end())  
    {
       return;
    }
    //if second airport does not exist disregard
    if(converter_.find(second) == converter_.end()) 
    {
        return;
    }
    //if an edge already exists between these two airports disregard
    if(areAdjacent(converter_.find(first)->second, converter_.find(second)->second) != NULL)
    {
        return;
    }
    //converts airport ID int to the actual airport Vertex
    Vertex first_vertex = converter_.find(first)->second;
    Vertex second_vertex = converter_.find(second)->second;

    //creates a new edge between first and second airports
    Edge e(&(adjacency_list_.find(first_vertex)->first), &(adjacency_list_.find(second_vertex)->first), weight);
    edge_list_.push_back(e);
    
    degree_map_[first_vertex]++;
    degree_map_[second_vertex]++;
    //updates edge list of both airports' adjacenecy lists
    adjacency_list_.find(first_vertex)->second.push_back(&edge_list_.back());
    adjacency_list_.find(second_vertex)->second.push_back(&edge_list_.back());


}

Graph::Edge* Graph::areAdjacent(const Vertex & v1, const Vertex & v2)
{
    
    if(degree_map_[v1] <= degree_map_[v2]) {
        // Search v1's adjacency list
        list<Edge *> adj = adjacency_list_.find(v1)->second;
        for(auto iter = adj.begin(); iter != adj.end(); ++iter) {
            Edge * edge = *iter;
            // Found corresponding edge
            if(edge->firstID_->airportID_ == v2.airportID_ || edge->secondID_->airportID_ == v2.airportID_) {
                return edge;
            } 
        }
    } else {

        // Search v2's adjacency list
        list<Edge *> adj = adjacency_list_.find(v2)->second;
        for(auto iter = adj.begin(); iter != adj.end(); ++iter) {
            Edge * edge = *iter;
            // Found corresponding edge
            if(edge->firstID_->airportID_ == v1.airportID_ || edge->secondID_->airportID_ == v1.airportID_) {
                return edge;
            } 
        }
    }
    return NULL;
}

list<Graph::Edge *> Graph::incidentEdges(const Vertex & v)
{
    return adjacency_list_[v]; //returns all the edges related to airport v
}

double Graph::getWeight(const Graph::Edge & e) {
    return e.weight_;
}

vector<Graph::Vertex> Graph::getVertices() {
    vector<Vertex> ret;
    //iterates through entire adjacency list to push all vertices into ret
    for(auto it = adjacency_list_.begin(); it != adjacency_list_.end(); ++it)
    {
        ret.push_back(it->first);
    }

    return ret;
}

vector<Graph::Edge *> Graph::getEdges() {
    if (edge_list_.empty()) {
        return vector<Graph::Edge *>();
    }

    vector<Edge*> ret;
    //iterates through entire edge list to push all edge pointers into ret
    for (auto it = edge_list_.begin(); it != edge_list_.end(); ++it) {
        ret.push_back(&(*it));
    }

    return ret;
}

vector<Graph::Vertex> Graph::getAdjacent(const Vertex & v) {
    vector<Vertex> ret;
    list<Graph::Edge *> edges = incidentEdges(v);

    for (auto it = edges.begin(); it != edges.end(); ++it) {
        //checks to see whether first airport of edge matches airport v
        if (*((*it)->firstID_) == v) {  
            //pushes other airport into vector
            ret.push_back(*((*it)->secondID_)); 
        } else {
            ret.push_back(*((*it)->firstID_));
        }
    }
    return ret; //returns all airport vertices related to airport v
}

Graph::v_label Graph::getVLabel(const Vertex & v) {
    return vertex_labels_[v];
}

void Graph::setVLabel(Vertex & v, v_label label) {
    vertex_labels_[v] = label;
}

Graph::e_label Graph::getELabel(const Edge *e) {
    return e->label_;
}

void Graph::setELabel(Edge *e, e_label label) {
    e->label_ = label;
}

void Graph::BFS() {
    for (Vertex & v : getVertices()) {
        setVLabel(v, UNEXPLORED);
    }

    for (Edge *e : getEdges()) {
        setELabel(e, UNDISCOVERED);
    }

    for (Vertex & v : getVertices()) {
        if (getVLabel(v) == UNEXPLORED) {
            BFS(v);
        }
    }
}

void Graph::BFS(Vertex & v) {
    queue<Vertex> q;
    setVLabel(v, VISITED);
    q.push(v);

    while (!q.empty()) {
        v = q.front();
        q.pop();
        // cout << v.airportID_ << " is labeled " << vertex_labels_[v] << endl;
        for (Vertex w : getAdjacent(v)) {
            if (getVLabel(w) == UNEXPLORED) {
                setELabel(areAdjacent(v, w), DISCOVERY);
                setVLabel(w, VISITED);
                q.push(w);
            } else if (getELabel(areAdjacent(v, w)) == UNDISCOVERED) {
                setELabel(areAdjacent(v, w), CROSS);
            }
        }
    }
    // for (auto it = edge_list_.begin(); it != edge_list_.end(); ++it) {
    //     cout << "Edge (" << it->firstID_->airportID_ << ", " << it->secondID_->airportID_ << ") is labeled " << it->label_ << endl;
    // }
}

void Graph::DFS() {
    for (Vertex & v : getVertices()) {
        setVLabel(v, UNEXPLORED);
    }

    for (Edge *e : getEdges()) {
        setELabel(e, UNDISCOVERED);
    }

    for (Vertex & v : getVertices()) {
        if (getVLabel(v) == UNEXPLORED) {
            DFS(v);
        }
    }
}

void Graph::DFS(Vertex & v) {
    stack<Vertex> s;
    setVLabel(v, VISITED);
    s.push(v);

    while (!s.empty()) {
        v = s.top();
        s.pop();
        // cout << v.airportID_ << " is labeled " << vertex_labels_[v] << endl;
        for (Vertex w : getAdjacent(v)) {
            if (getVLabel(w) == UNEXPLORED) {
                setELabel(areAdjacent(v, w), DISCOVERY);
                setVLabel(w, VISITED);
                s.push(w);
            } else if (getELabel(areAdjacent(v, w)) == UNDISCOVERED) {
                setELabel(areAdjacent(v, w), CROSS);
            }
        }
    }
    // for (auto it = edge_list_.begin(); it != edge_list_.end(); ++it) {
    //     cout << "Edge (" << it->firstID_->airportID_ << ", " << it->secondID_->airportID_ << ") is labeled " << it->label_ << endl;
    // }
}

void Graph::printVertex()
{
    for(auto it = adjacency_list_.begin(); it != adjacency_list_.end(); it++)
    {
        std::cout << it->first.airportID_ << " has degree " << degree_map_[it->first] << ". ";
        std::cout << it->first.airportID_ << " is adjacent to: " << std::endl;
        
        list<Edge *> adj = incidentEdges(it->first);
        for(Edge * edge : adj) {
            std::cout << "(" << edge->firstID_->airportID_ << ", " << edge->secondID_->airportID_ << ") " << std::endl;
        }
    }
}

void Graph::printEdge()
{
    for(auto it = edge_list_.begin(); it != edge_list_.end(); it++)
    {
        std::cout << it->firstID_->airportID_ << " connected to " << it->secondID_->airportID_ << ". Weight = " << it->weight_ << std::endl;
    }
    std::cout << "Number of airports: " << adjacency_list_.size() << std::endl;
}

list<Graph::Vertex> Graph::Dijkstra(int sourceID, int destID)
{
    // Source Vertex
    Vertex v_source = converter_.find(sourceID)->second;
    // Destination Vertex
    Vertex v_dest = converter_.find(destID)->second;

    // The shortest path between source and destination vertex.
    list<Vertex> ret;
    // Defined struct to be the element inside priority_queue.
    typedef struct pq_elem
    {
        Vertex v_;
        double dist_;

        pq_elem(Vertex v, double dist)
        {
            v_ = v;
            dist_ = dist;
        }
        
        bool operator>(const pq_elem & other) const
        {
            return dist_ > other.dist_;
        }
        
        bool operator<(const pq_elem & other) const
        {
            return dist_ < other.dist_;
        }

        bool operator==(const pq_elem & other) const
        {
            return dist_ == other.dist_;
        }
    }pq_elem;

    map<Vertex, double> vertex_to_dist;
    map<Vertex, Vertex> vertex_to_prev;
    map<Vertex, bool> vertex_to_visited;

    // dist[source] ← 0
    vertex_to_dist.insert(std::pair<Vertex, double>(v_source, 0));
    vertex_to_prev.insert(std::pair<Vertex, Vertex>(v_source, v_source));
    vertex_to_visited.insert(std::pair<Vertex, bool>(v_source, false));

    // create vertex priority queue Q
    std::priority_queue <pq_elem, vector<pq_elem>, std::greater<pq_elem>> Q;

    // for each vertex v in Graph:
    vector<Vertex> all_v = getVertices();
    for(auto it = all_v.begin(); it != all_v.end(); it++)
    { 
        // if v ≠ source
        if(!(*it == v_source))
        {
            // dist[v] ← INFINITY
            vertex_to_dist.insert(std::pair<Vertex, double>(*it, std::numeric_limits<double>::infinity()));
            // prev[v] ← UNDEFINED
            vertex_to_prev.insert(std::pair<Vertex, Vertex>(*it, *it));
            vertex_to_visited.insert(std::pair<Vertex, bool>(*it, false));
        }
        Q.push(pq_elem(*it, vertex_to_dist.find(*it)->second));
    }
    // while Q is not empty:
    while(Q.empty() == false)
    {
        // u ← Q.extract_min()
        pq_elem u = Q.top();
        Q.pop();
        

        // if p ≤ dist[u]
        if(u.dist_ <= vertex_to_dist.find(u.v_)->second)
        {
            vertex_to_visited.find(u.v_)->second = true;

                        
            // TERMINAL CONDITION
            // if u = target
            if(u.v_ == v_dest)
            {
                break;
            }
            
            // for each neighbor v of u:
            for(Vertex & v : getAdjacent(u.v_))
            {
                if(vertex_to_visited.find(v)->second == false)
                {
                    // alt ← dist[u] + length(u, v)
                    double alt = vertex_to_dist.find(u.v_)->second + areAdjacent(u.v_, v)->weight_;

                    // if alt < dist[v]
                    if(alt < vertex_to_dist.find(v)->second)
                    {
                        // dist[v] ← alt
                        vertex_to_dist.find(v)->second = alt;
                        // prev[v] ← u
                        vertex_to_prev.find(v)->second = u.v_;
                        // Q.decrease_priority(v, alt)
                        Q.push(pq_elem(v, alt));
                    }
                }
            }


        }
    }
    
    Vertex curr = v_dest;
    while(vertex_to_prev.find(curr)->second != vertex_to_prev.find(curr)->first)
    {
       ret.push_front(curr);
       curr = vertex_to_prev.find(curr)->second;
    }
    ret.push_front(v_source);

    // Print the phyiscal distance of the shortest path.
    std::cout << "Dijkstra Output::Shortest Path Physical Distance: " << vertex_to_dist.find(v_dest)->second << std::endl;

    // Print all the vertices in the shortest path from source to destination.
    std::cout << "Dijkstra Output::Vertices on Shortest Path: " << std::endl;
    int counter = 1;
    for(auto it = ret.begin(); it != ret.end(); ++it)
    {
        std::cout << "Vertex " << counter << " with ID: " << it->airportID_ << std::endl;
        counter++;

    }
    return ret;
    
}
list<Graph::Vertex> Graph::landmark(int sourceID, int stopID, int destID){
    list<Vertex> first_route=Dijkstra(sourceID,stopID); //shortest route from source to stop
    list<Vertex>second_route=Dijkstra(stopID,destID);   //shortest route from stop to destination
    second_route.pop_front();
    for(auto it=second_route.begin();it!=second_route.end();++it) {
        first_route.push_back(*it); //combining second route to first to create path with detour
    }
    std::cout<<"Landmark path from "<<sourceID<<" to "<<destID<<" through "<<stopID<<": ";
    for(auto it=first_route.begin();it!=first_route.end();++it){
        std::cout<<it->airportID_<<" ";
    }
    std::cout<<endl;
    return first_route;
}