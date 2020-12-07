#pragma once 
#include "Graph.h"
#include <iostream>
#include <list>
using std::list;

/* This file contains headers for functions that can be performed in the user menu */

//prints out an outline of the main menu and provides user with options to choose from
void print_main_menu();

// prompts the user to input two airports to find the shortest route between them and prints the result
void shortestPath(Graph & g);

// prompts the user to input two airports to find the shortest route between
// them while also reaching another airport and prints the result
void shortestPath_landmark(Graph & g);

//prints all the vertices and edges of a given graph
void display_graph(Graph & g);

//calls a BFS traversal on given graph
void perform_BFS(Graph & g);

//calls a DFS traversal on given graph
void perform_DFS(Graph & g);

//prompts the user to input two airports and checks to see if an edge exists between the two
void connection_exists(Graph & g);

//prompts the user for an airport and prints the total number of routes from that airport
void num_routes(Graph & g);

// prints the information of an airport the user wants to know about
void cool_info(Graph & g);

// prints information displaying that Dijkstra's agorithm works on a graph
void dijkstra_test(Graph & g);

// prints information displaying that Landmark path algorithm works on a graph
void landmark_test(Graph & g);