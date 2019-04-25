#ifndef DATASET_LOADER_H
#define DATASET_LOADER_H

#include <vector>
#include <algorithm>
#include <string>
#include <utility>
#include <sstream>
#include <iterator>
#include <iostream>
#include <fstream>
#include <math>

using namespace std;

// General useful functions
vector< string > split_line( const string& line );

struct dataset_loader 
{
    vector< pair<int, int> > points; // Coordinates of the points
    vector< vector<int> > adjacency_matrix; // Adjacency Matrix
    vector< int > demands; // Node demand

    string instance_file, instance_name;

    int dimension, depot_index, uniform_vehicle_capacity;

    // Function for calculating euclidean distance
    int euclidean_distance( const pair<int, int>& a, const pair<int, int>& b);
    
    // Function for loading the instance
    void load_instance( const string filename );
    
    // Function for initializing distances on the adjacency_matrix
    void adjacency_matrix_initialization();
    
    void print_graph();
    
    // Constructor
    dataset_loader( string _instance_file );
    
    
};

#endif
