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

vector< string > split_line( string& line );

struct instance 
{
    vector< pair<int, int> > points;
    vector<int> demands;

    string path_to_instance;
    string instance_name;

    int dimension, depot_index, uniform_vehicle_capacity;

    instance( string _path_to_instance );

    instance();

};

#endif
