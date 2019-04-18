#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <utility>
#include <iterator>
#include <fstream>

using namespace std;

struct instance 
{
    vector< pair<int, int> > points;
    vector<int> demands;
    vector< vector<int> > adjacency_matrix;

    string path_to_instance;
    string instance_name;

    int dimension, depot_index, uniform_vehicle_capacity;
    
    void initialize_adjacency_matrix();

    instance( string _path_to_instance );

    instance();

};

#endif
