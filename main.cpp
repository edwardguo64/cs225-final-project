#include "menu_helpers.h"
#include <iostream>
#include <string>
using std::string;

int main()
{
    Graph g("data/airports.dat", "data/routes.dat");

    Graph g_sublist("test_airports.dat", "test_routes.dat");

    Graph test;
    test.insertVertex(Graph::Vertex(1, "", "", "", "", "", 0, 0));
    test.insertVertex(Graph::Vertex(2, "", "", "", "", "", 0, 0));
    test.insertVertex(Graph::Vertex(3, "", "", "", "", "", 0, 0));
    test.insertVertex(Graph::Vertex(4, "", "", "", "", "", 0, 0));
    test.insertVertex(Graph::Vertex(5, "", "", "", "", "", 0, 0));
    test.insertVertex(Graph::Vertex(6, "", "", "", "", "", 0, 0));
    test.insertEdge(1,2,3);
    test.insertEdge(2,3,1);
    test.insertEdge(1,3,5);
    test.insertEdge(1,5,5);
    test.insertEdge(2,4,2);
    test.insertEdge(3,4,1);
    test.insertEdge(3,5,1);
    test.insertEdge(4,5,3);
    test.insertEdge(4,6,2);
    test.insertEdge(5,6,4);

    string choice;
    print_main_menu();
    bool valid = true;
    
    do {
        cout << "\nChoice: ";
        cin >> choice;

        valid = true;
        for (size_t i = 0; i < choice.size(); i++) {
            if (!isdigit(choice[i])) {
                valid = false;
            }
        }

        if (!valid) {
            cout << "\nThat is an invalid choice, please try again." << endl;
            print_main_menu();
        }
    } while (!valid);

	while (std::stoi(choice) != 12)
	{
		switch (std::stoi(choice))
		{
		case 1:
            display_graph(g_sublist);
			print_main_menu();
			break;
		case 2:
            perform_BFS(g_sublist);
			print_main_menu();
			break;
		case 3:
            perform_DFS(g_sublist);
			print_main_menu();
			break;
        case 4:
            dijkstra_test(test);
            print_main_menu();
            break;
        case 5:
            landmark_test(test);
            print_main_menu();
            break;
		case 6:
            shortestPath(g);
			print_main_menu();
			break;
		case 7:
            shortestPath_landmark(g);
			print_main_menu();
			break;
		case 8:
            connection_exists(g);
			print_main_menu();
			break;
        case 9:
            num_routes(g);
            print_main_menu();
            break;
        case 10:
            list_routes(g);
            print_main_menu();
            break;
        case 11:
            cool_info(g);
            print_main_menu();
            break;
		default:
			cout << "\nThat is an invalid choice, please try again." << endl;
			print_main_menu();
			break;
		}

		do {
            cout << "\nChoice: ";
            cin >> choice;

            valid = true;
            for (size_t i = 0; i < choice.size(); i++) {
                if (!isdigit(choice[i])) {
                    valid = false;
                }
            }

            if (!valid) {
                cout << "\nThat is an invalid choice, please try again." << endl;
                print_main_menu();
            }
        } while (!valid);
	}
	return 0;
}
