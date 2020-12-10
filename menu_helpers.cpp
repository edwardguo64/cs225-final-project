#include "menu_helpers.h"

void print_main_menu()
{
	cout << "\n                   Main Menu" << endl << endl;
	cout << "    1) Display the graph of the sublist of airports" << endl;
	cout << "    2) Perform BFS on sublist of airports" << endl;
	cout << "    3) Perform DFS on sublist of airports" << endl;
    cout << "    4) Verify Djikstra's algorithm on sample test graph" << endl;
    cout << "    5) Verify Landmark algorithm on sample test graph" << endl;
	cout << "    6) Find shortest route between two airports" << endl;
	cout << "    7) Find shortest route between two airports that passes through another airport" << endl;
	cout << "    8) Does a direct flight exist between these two airports" << endl;
	cout << "    9) Number of routes from an airport" << endl;
    cout << "   10) List out all flights that connect with an airport" << endl;
    cout << "   11) Find out more information about a particular airport" << endl;
	cout << "   12) Quit" << endl;
}

void shortestPath(Graph & g) {
    string icao1, icao2;

    // prompts the user to input two airports, looping to ensure that the user inputs valid ICAO codes
    cout << "\nEnter the two airports you want to find the shortest route between: " << endl;
    do {
        cout << "Airport ICAO 1: ";
        cin >> icao1;

        cout << "Airport ICAO 2: ";
        cin >> icao2;

        if (g.getID(icao1) == -1) {
			cout << "\nThe first airport does not exist in this dataset. Try again:" << endl;
		} else if(g.getID(icao2) == -1) {
			cout << "\nThe second airport does not exist in this dataset. Try again:" << endl;
        }
    } while (g.getID(icao1) == -1 || g.getID(icao2) == -1);

    int id1 = g.getID(icao1), id2 = g.getID(icao2);

    // performs Dijstra's algorithm to find the shortest route and record the total distance
    double distance = 0;
    list<Graph::Vertex> route = g.Dijkstra(id1, id2, &distance);

    // displays the result
    if (route == list<Graph::Vertex>()) {
        cout << "\nThe airports are not connected in this dataset. No path was found between the two airports" << endl;
    } else {
        cout << "\nThe shortest route between " << icao1 << " and " << icao2 << " go through the following airports:" << endl << endl;
        for (Graph::Vertex &v : route) {
            cout << v.ICAO_ << endl;
        }
        cout << "\nThe distance between the two airports is " << std::trunc(distance) << " km." << endl;
	}
}

void shortestPath_landmark(Graph & g) {
    string icao1, icao2, icao_stop;

    // prompts the user to input three airports, looping to ensure that the user inputs valid ICAO codes
    cout << "\nEnter the two airports you want to find the shortest route between and the airport you want to pass along: " << endl;
    do {
        cout << "Airport ICAO 1: ";
        cin >> icao1;

        cout << "Airport ICAO 2: ";
        cin >> icao2;

        cout << "Enter the airport you want to pass along the route: " << endl;
        cout << "Airport ICAO: ";
		cin >> icao_stop;

		if (g.getID(icao1) == -1)
		{
			cout << "\nThe first airport does not exist in this dataset. Try again:" << endl;
		}
		else if (g.getID(icao2) == -1)
		{
			cout << "\nThe second airport does not exist in this dataset. Try again:" << endl;
		}
        else if (g.getID(icao_stop) == -1)
		{
			cout << "\nThe landmark airport does not exist in this dataset. Try again:" << endl;
		}
	} while (g.getID(icao1) == -1 || g.getID(icao2) == -1 || g.getID(icao_stop) == -1);

    int id1 = g.getID(icao1), id2 = g.getID(icao2), id_stop = g.getID(icao_stop);

    // performs landmark path algorithm to find the shortest route and record the total distance
    double distance = 0;
    list<Graph::Vertex> route = g.landmark(id1, id_stop, id2, &distance);

    // displays the result
    if (route == list<Graph::Vertex>()) {
        cout << "\nThe airports are not connected in this dataset. No path was found between the two airports through the landmark" << endl;
    } else {
        cout << "\nThe shortest route between " << icao1 << " and " << icao2 << " through " << icao_stop <<  " go through the following airports:" << endl << endl;
        for (Graph::Vertex &v : route) {
            cout << v.ICAO_ << endl;
        }
        cout << "\nThe distance between the two airports through the landmark is " << std::trunc(distance) << " km." << endl;
	}
}

void display_graph(Graph & g){
   g.printVertex();
   g.printEdge(); 
}

void perform_BFS(Graph & g){
    g.BFS();
}

void perform_DFS(Graph & g){
    g.DFS();
}

void connection_exists(Graph & g){
    string icao1, icao2;
    
    // prompts the user to input two airports, looping to ensure that the user inputs valid ICAO codes
    cout << "\nEnter the two airports you want to check if there is a direct route between: " << endl;
    do {
        cout << "Airport ICAO 1: ";
        cin >> icao1;

        cout << "Airport ICAO 2: ";
        cin >> icao2;

		if (g.getID(icao1) == -1)
		{
			cout << "\nThe first airport does not exist in this dataset. Try again:" << endl;
		}
		else if (g.getID(icao2) == -1)
		{
			cout << "\nThe second airport does not exist in this dataset. Try again:" << endl;
		}
	} while (g.getID(icao1) == -1 || g.getID(icao2) == -1);

    Graph::Vertex v1 = g.ID_to_Vertex(g.getID(icao1));
    Graph::Vertex v2 = g.ID_to_Vertex(g.getID(icao2));
    
    // displays the information if the two airports are connected or not with a direct route
    if(g.areAdjacent(v1, v2)) {
        cout << "\nThere exists a direct flight between " << icao1 << " and " << icao2 << " in this dataset." << endl;
    } else {
        cout << "\nThere does not exist a direct flight between " << icao1 << " and " << icao2 << " in this dataset." << endl;
    }
}

void num_routes(Graph & g){
    string icao;

    // prompts the user to input an airport, looping to ensure that the user inputs a valid ICAO code
    cout << "\nEnter the airport you want to find the number of routes from: " << endl;
    do {
        cout << "Airport ICAO: ";
        cin >> icao;

        if (g.getID(icao) == -1) {
			cout << "\nThe airport does not exist in this dataset. Try again:" << endl;
		} 
    } while (g.getID(icao) == -1);

    int id1 = g.getID(icao);
	vector<Graph::Vertex> adjacent;
    Graph::Vertex v = g.ID_to_Vertex(id1);
    
    // displays the number of adjacent edges from that airport as routes
    adjacent=g.getAdjacent(v);
	cout << "\nThis airport has " << adjacent.size() << " routes in this dataset."<< endl;
}

void list_routes(Graph & g){
    string icao;

    // prompts the user to input an airport, looping to ensure that the user inputs a valid ICAO code
    cout << "\nEnter the airport you want to see the list of routes for: " << endl;
    do {
        cout << "Airport ICAO: ";
        cin >> icao;

        if (g.getID(icao) == -1) {
			cout << "\nThe airport does not exist in this dataset. Try again:" << endl;
		} 
    } while (g.getID(icao) == -1);

    vector<Graph::Vertex> adjacent = g.getAdjacent(g.ID_to_Vertex(g.getID(icao)));
    
    cout << "\nThe airports that have direct connections in this dataset from the airport " << icao << " are:" << endl << endl;
    for (size_t i = 0; i < adjacent.size(); i++){
        if (i == adjacent.size() - 1) {
            cout << adjacent[i].ICAO_ << endl;
        } else {
            if(i != 0 && i % 15 == 0) cout << endl; // prints out 15 airport ICAOs per line
            cout << adjacent[i].ICAO_ << ", ";
        }
	}
}

void cool_info(Graph & g)
{
    string icao;

    // prompts the user to input an airport, looping to ensure that the user inputs a valid ICAO code
    cout << "\nEnter the airport you want to learn more information about: " << endl;
    do {
    
        cout << "Airport ICAO: ";
        cin >> icao;
        
        if (g.getID(icao) == -1) {
			cout << "\nThe airport does not exist in this dataset. Try again:" << endl;
		}
    } while (g.getID(icao) == -1);

    Graph::Vertex v = g.ID_to_Vertex(g.getID(icao));

    // displays the airport information
    cout << "\nAirport Name: " << v.name_ << endl;
    cout << "Airport is located in: " << v.city_ << ", " << v.country_ << endl;
    cout << "Airport has IATA code: " << v.IATA_ << endl;
    cout << "Airport has ICAO code: " << v.ICAO_ << endl;
    cout << "Airport has OpenFlights ID: " << v.airportID_ << endl;
    cout << "Airport Latitude: " << v.latitude_ << endl;
    cout << "Airport Longitude: " << v.longitude_ << endl;
}

void dijkstra_test(Graph & g) {
    // Following code tests Dijkstra's algorithm based on a small hand-drawn graph.
    double distance = 0;

    g.printVertex();
    g.printEdge();
    list<Graph::Vertex> land_list;
    land_list=g.Dijkstra(1,6, &distance);  
    cout << "Test 1: Shortest path from 1 to 6" << endl;
    for(auto it=land_list.begin();it!=land_list.end();++it){
        cout<<g.Vertex_to_ID(*it)<<" ";
    }
    cout << "\nThe distance between the two vertices is: " << distance << endl;
    cout << endl;
    land_list=g.Dijkstra(1,2, &distance);  
    cout << "Test 1: Shortest path from 1 to 2" << endl;
    for(auto it=land_list.begin();it!=land_list.end();++it){
        cout<<g.Vertex_to_ID(*it)<<" ";
    }
    cout << "\nThe distance between the two vertices is: " << distance << endl;
    cout << endl;
    land_list=g.Dijkstra(3,6, &distance);  
    cout << "Test 1: Shortest path from 3 to 6" << endl;
    for(auto it=land_list.begin();it!=land_list.end();++it){
        cout<<g.Vertex_to_ID(*it)<<" ";
    }
    cout << "\nThe distance between the two vertices is: " << distance << endl;
    cout << endl;
    land_list=g.Dijkstra(2,5, &distance);  
    cout << "Test 1: Shortest path from 2 to 5" << endl;
    for(auto it=land_list.begin();it!=land_list.end();++it){
        cout<<g.Vertex_to_ID(*it)<<" ";
    }
    cout << "\nThe distance between the two vertices is: " << distance << endl;
    cout << endl;
    
    // Correctly returns path of length 0 with 1 vertex on the path.
    land_list=g.Dijkstra(1,1, &distance);  
    cout << "Test 1: Shortest path from 1 to 1" << endl;
    for(auto it=land_list.begin();it!=land_list.end();++it){
        cout<<g.Vertex_to_ID(*it)<<" ";
    }
    cout << "\nThe distance between the two vertices is: " << distance << endl;
    cout << endl;
    land_list=g.Dijkstra(6,6, &distance); 
    cout << "Test 1: Shortest path from 6 to 6" << endl;
    for(auto it=land_list.begin();it!=land_list.end();++it){
        cout<<g.Vertex_to_ID(*it)<<" ";
    }
    cout << "\nThe distance between the two vertices is: " << distance << endl;
    cout << endl;
}

void landmark_test(Graph & g) {
    double distance = 0;
	list<Graph::Vertex> land_list;

    //provides information on the predetermined graph
	g.printVertex();
	g.printEdge();
    
    //performs several landmark tests on a predetermined graph
    land_list=g.landmark(2,5,4, &distance);  
    cout << "Test 1: Go from 2 to 4, through 5" << endl;
    for(auto it=land_list.begin();it!=land_list.end();++it){
        cout<<g.Vertex_to_ID(*it)<<" ";
    }
	cout << endl;
	land_list = g.landmark(1, 6, 9, &distance);
	cout << "Test 2: Go from 1 to 9, through 6. Should not be valid, test for disconnected components." << endl;
    for(auto it=land_list.begin();it!=land_list.end();++it){
        cout<<g.Vertex_to_ID(*it)<<" ";
    }
	cout << endl;
	land_list = g.landmark(1, 6, 3, &distance);
	cout << "Test 3: Go from 1 to 3, through 6." << endl;
    for(auto it=land_list.begin();it!=land_list.end();++it){
        cout<<g.Vertex_to_ID(*it)<<" ";
    }
    cout << endl;
}