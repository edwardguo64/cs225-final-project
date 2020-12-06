#pragma once 
#include "Graph.h"
#include <iostream>
#include <list>
using std::list;

/* This file contains headers for functions that can be performed in the user menu */

void print_main_menu();
void shortestPath(Graph & g);
void shortestPath_landmark(Graph & g);
void display_graph(Graph & g);
void perform_BFS(Graph & g);
void perform_DFS(Graph & g);
void connection_exists(Graph & g);
void num_routes(Graph & g);
void cool_info(Graph & g);
void djikstra_test(Graph & g);
void landmark_test(Graph & g);