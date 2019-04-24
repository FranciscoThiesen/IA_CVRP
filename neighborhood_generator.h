#ifndef NEIGHBORHOOD_GENERATOR_H
#define NEIGHBORHOOD_GENERATOR_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdio>
#include "data_loader.h"

struct neighborhood_generator {
    instance data_inst;

    void update_solution(vector<vector<int>> &updated_routes, vector<int> &updated_route_capacities);
    
    neighborhood_generator(instance inst);
    neighborhood_generator();
};

#endif