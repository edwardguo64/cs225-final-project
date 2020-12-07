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
                name = data.substr(1);
                // Check if the name has a comma in it
                while (data[data.size() - 1] != '\"') { //until the last char of data is "
                    getline(file, data, ',');           //read and store next piece of comma-separated string 
                    name += "," + data;                 //concatenate to include entire string between " "
                }
                name.pop_back();
            }
            else if(counter == 2){
                city = data.substr(1);
                // Check if the city has a comma in it
                while (data[data.size() - 1] != '\"') {
                    getline(file, data, ',');
                    name += "," + data;
                }
                city.pop_back();
            }
            else if(counter == 3){
                country = data.substr(1);
                // Check if the country has a comma in it
                while (data[data.size() - 1] != '\"') {
                    getline(file, data, ',');
                    name += "," + data;
                }
				country.pop_back();
			}
			else if(counter == 4){
                IATA = data.substr(1);
				IATA.pop_back();
			}
			else if(counter== 5){
                ICAO = data.substr(1);
                ICAO.pop_back();
            }
            else if(counter == 6){
                latitude = std::stod(data);
            }
            else if(counter == 7){
                longitude = std::stod(data);
                insertVertex(Vertex(airportID, name, city, country, IATA, ICAO, latitude, longitude));
                insertICAO(ICAO, airportID);
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
                Vertex first_vertex = id_to_vertex_.find(firstID)->second;
                Vertex second_vertex = id_to_vertex_.find(secondID)->second;
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

void Graph::insertICAO(string ICAO, int id) {
    ICAO_map_.insert(std::pair<string, int>(ICAO, id));         // inserts pair of string and airport ID into ICAO map
}

void Graph::insertVertex(Vertex v)
{
    std::pair<Vertex, list<Edge *>> ret(v, list<Edge *>());     // create pair of Vertex v and list of edge pointers
    adjacency_list_.insert(ret);                                // insert the pair to the adjacency list
    std::pair<int, Vertex> ret2(v.airportID_, v);               // create pair of int airport ID of v and Vertex v
    id_to_vertex_.insert(ret2);                                    // insert the pair to the converter map
}

// Needs to check if either (u, v) or (v, u) is in the graph already.
void Graph::insertEdge(int first, int second, double weight)
{
    //if first airport does not exist disregard
    if(id_to_vertex_.find(first) == id_to_vertex_.end())  
    {
       return;
    }
    //if second airport does not exist disregard
    if(id_to_vertex_.find(second) == id_to_vertex_.end()) 
    {
        return;
    }
    //if an edge already exists between these two airports disregard
    if(areAdjacent(id_to_vertex_.find(first)->second, id_to_vertex_.find(second)->second) != NULL)
    {
        return;
    }
    //converts airport ID int to the actual airport Vertex
    Vertex first_vertex = id_to_vertex_.find(first)->second;
    Vertex second_vertex = id_to_vertex_.find(second)->second;

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
    return e.weight_;   // returns the weight of the edge
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
    if (edge_list_.empty()) {                   // if the edge list is empty
        return vector<Graph::Edge *>();         // return a default vector
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
    return vertex_labels_[v];       // returns the vertex label of the vertex v
}

void Graph::setVLabel(Vertex & v, v_label label) {
    vertex_labels_[v] = label;      // sets the vertex label of the vertex v
}

Graph::e_label Graph::getELabel(const Edge *e) {
    return e->label_;               // returns the edge label of the edge e
}

void Graph::setELabel(Edge *e, e_label label) {
    e->label_ = label;              // sets the edge label of the edge e
}

int Graph::getID(string ICAO) {
    if (ICAO_map_.find(ICAO) == ICAO_map_.end()) {      // if the ICAO string does not exist in the data set
        return -1;                                      // return -1
    }
    return ICAO_map_[ICAO];                             // return the airport ID
}

void Graph::BFS() {
    for (Vertex & v : getVertices()) {              // sets all verticies as UNEXPLORED
        setVLabel(v, UNEXPLORED);
    }

    for (Edge *e : getEdges()) {                    // sets all edges as UNDISCOVERED
        setELabel(e, UNDISCOVERED);
    }

    for (Vertex & v : getVertices()) {              // performs BFS on all verticies that are UNEXPLORED
        if (getVLabel(v) == UNEXPLORED) {
            BFS(v);
        }
    }
}

void Graph::BFS(Vertex & v) {
    queue<Vertex> q;                                                    // declares queue for BFS
    setVLabel(v, VISITED);                                              // sets the label of the vertex to VISITED
    q.push(v);                                                          // pushes the vertex into the queue

    while (!q.empty()) {                                                // while the queue is not empty
        v = q.front();                                                  // the vertex is set to the queue's front
        q.pop();                                                        // pop the front of the queue
        cout << v.airportID_ << " is labeled " << vertex_labels_[v] << endl;
        for (Vertex w : getAdjacent(v)) {                               // for all the vertices w that are adjacent to the vertex v
            if (getVLabel(w) == UNEXPLORED) {                           // if the vertex w is UNEXPLORED
                setELabel(areAdjacent(v, w), DISCOVERY);                // set the label of edge to be DISCOVERY
                setVLabel(w, VISITED);                                  // set the label of the vertex VISITED
                q.push(w);                                              // pushes the vertex w to the queue
            } else if (getELabel(areAdjacent(v, w)) == UNDISCOVERED) {  // else if the edge is UNDISCOVERED
                setELabel(areAdjacent(v, w), CROSS);                    // sets the label of the edge to be CROSS
            }
        }
    }
    // Display BFS
    for (auto it = edge_list_.begin(); it != edge_list_.end(); ++it) {
        cout << "Edge (" << it->firstID_->airportID_ << ", " << it->secondID_->airportID_ << ") is labeled " << it->label_ << endl;
    }
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

    // Refer to BFS for explanations on algorithm. DFS is the same algorithm but with use of a stack instead of a queue.
    stack<Vertex> s;
    setVLabel(v, VISITED);
    s.push(v);

    while (!s.empty()) {
        v = s.top();
        s.pop();
        cout << v.airportID_ << " is labeled " << vertex_labels_[v] << endl;
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
    // Display DFS
    for (auto it = edge_list_.begin(); it != edge_list_.end(); ++it) {
        cout << "Edge (" << it->firstID_->airportID_ << ", " << it->secondID_->airportID_ << ") is labeled " << it->label_ << endl;
    }
}

void Graph::printICAOmap() {
    for (auto it = ICAO_map_.begin(); it != ICAO_map_.end(); ++it) {
        cout << it->first << " is mapped to " << it->second << endl;
    }
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

list<Graph::Vertex> Graph::Dijkstra(int sourceID, int destID, double * distance)
{
    // Source Vertex
    Vertex v_source = id_to_vertex_.find(sourceID)->second;
    // Destination Vertex
    Vertex v_dest = id_to_vertex_.find(destID)->second;

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

    // dist[source] = 0
    vertex_to_dist.insert(std::pair<Vertex, double>(v_source, 0));
    vertex_to_prev.insert(std::pair<Vertex, Vertex>(v_source, v_source));
    vertex_to_visited.insert(std::pair<Vertex, bool>(v_source, false));

    // create vertex priority queue Q
    std::priority_queue <pq_elem, vector<pq_elem>, std::greater<pq_elem>> Q;

    // for each vertex v in Graph:
    vector<Vertex> all_v = getVertices();
    for(auto it = all_v.begin(); it != all_v.end(); it++)
    { 
        // if v = source
        if(!(*it == v_source))
        {
            // dist[v] = INFINITY
            vertex_to_dist.insert(std::pair<Vertex, double>(*it, std::numeric_limits<double>::infinity()));
            // prev[v] = UNDEFINED
            vertex_to_prev.insert(std::pair<Vertex, Vertex>(*it, *it));
            vertex_to_visited.insert(std::pair<Vertex, bool>(*it, false));
        }
        Q.push(pq_elem(*it, vertex_to_dist.find(*it)->second));
    }
    // while Q is not empty:
    while(Q.empty() == false)
    {
        // u = Q.extract_min()
        pq_elem u = Q.top();
        Q.pop();
        

        // if p <= dist[u]
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
                    // alt = dist[u] + length(u, v)
                    double alt = vertex_to_dist.find(u.v_)->second + areAdjacent(u.v_, v)->weight_;

                    // if alt < dist[v]
                    if(alt < vertex_to_dist.find(v)->second)
                    {
                        // dist[v] = alt
                        vertex_to_dist.find(v)->second = alt;
                        // prev[v] = u
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
        if(vertex_to_prev.find(curr) == vertex_to_prev.end()){
			//std::cout << "The two airports are not connected." << endl;
			return list<Vertex>();
		}
	    curr = vertex_to_prev.find(curr)->second;
    }
    ret.push_front(v_source);

    // Print the phyiscal distance of the shortest path.
    //std::cout << "Dijkstra Output::Shortest Path Physical Distance: " << vertex_to_dist.find(v_dest)->second << std::endl;
    *distance = vertex_to_dist.find(v_dest)->second;
    // Print all the vertices in the shortest path from source to destination.
    //std::cout << "Dijkstra Output::Vertices on Shortest Path: " << std::endl;
    int counter = 1;
    for(auto it = ret.begin(); it != ret.end(); ++it)
    {
        //std::cout << "Vertex " << counter << " with ID: " << it->airportID_ << std::endl;
        counter++;

    }
    return ret;
    
}
list<Graph::Vertex> Graph::landmark(int sourceID, int stopID, int destID, double *tot_distance){
    double distance1 = 0;
    double distance2 = 0;
    list<Vertex> first_route=Dijkstra(sourceID,stopID, &distance1); //shortest route from source to stop
    list<Vertex>second_route=Dijkstra(stopID,destID, &distance2);   //shortest route from stop to destination
    // Testing for unconnected graph components
    if(first_route==list<Vertex>()||second_route==list<Vertex>()){
		//std::cout << "Airports are not connected, path not found from " <<sourceID <<" to " <<destID<< " through " <<stopID<< endl;
		return list<Vertex>();
	}
    *tot_distance = distance1 + distance2;
	second_route.pop_front();
    for(auto it=second_route.begin();it!=second_route.end();++it) {
        first_route.push_back(*it); //combining second route to first to create path with detour
    }
    //std::cout<<"Landmark path from "<<sourceID<<" to "<<destID<<" through "<<stopID<<": ";
    for(auto it=first_route.begin();it!=first_route.end();++it){
        //std::cout<<it->airportID_<<" ";
    }
    //std::cout<<endl;
    return first_route;
}

Graph::Vertex Graph::ID_to_Vertex(int portID){
	return id_to_vertex_.find(portID)->second;     //returns respective Vertex given airport ID
}

int Graph::Vertex_to_ID(Vertex v){
    return v.airportID_;                        //returns Vertex's airport ID
}