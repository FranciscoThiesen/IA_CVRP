#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdio>
#include "data_loader.h"
#include "neighborhood_generator.h"

int route_cost(vector<int> route, instance data_inst) {
    int cost = 0;
    for (int i = 0; i < route.size() - 1; i++) {
        int dist = data_inst.adjacency_matrix[route[i]][route[i+1]];
        cost += dist;
    }
    int dist_origin = data_inst.adjacency_matrix[route[route.size() - 1]][data_inst.depot_index];
    cost += dist_origin;
    return cost;
}

// EXCHANGE
void swap_cities(vector<vector<int>> &updated_routes, int route1, int route2, int idx1, int idx2) {
    int temp = updated_routes[route1][idx1];
    updated_routes[route1][idx1] = updated_routes[route2][idx2];
    updated_routes[route2][idx2] = temp;
}

void exchange(vector<vector<int>> &updated_routes, vector<int> &updated_routes_capacities, instance data_inst) {
    int did_exchange = 0;
    while (!did_exchange) {
        // randomly select index two indexes to swap - do not allow index 0
        int route1 = rand() % updated_routes.size();
        int route2 = rand() % updated_routes.size();
        int idx1 = (rand() % (updated_routes[route1].size() - 1)) + 1;
        int idx2 = (rand() % (updated_routes[route2].size() - 1)) + 1;
        
        // find the cities in those indexes
        int city_idx1 = updated_routes[route1][idx1];
        int city_idx2 = updated_routes[route2][idx2];
        
        if (route1 != route2 || idx1 != idx2) { // randomly selected nodes are different
            // exchange within the same route - no need to check for capacity constraints
            if (route1 == route2) {
                swap_cities(updated_routes, route1, route2, idx1, idx2);
                did_exchange = 1;
            }
            // exchange within different routes - need to check if capacity is exceeded
            else {
                int capacity_route1 = updated_routes_capacities[route1];
                int capacity_route2 = updated_routes_capacities[route2];
                
                int updated_capacity_route1 = capacity_route1 - data_inst.demands[city_idx1] + data_inst.demands[city_idx2];
                int updated_capacity_route2 = capacity_route2 - data_inst.demands[city_idx2] + data_inst.demands[city_idx1];
                
                if (updated_capacity_route1 < data_inst.uniform_vehicle_capacity &&
                    updated_capacity_route2 < data_inst.uniform_vehicle_capacity) {
                    updated_routes_capacities[route1] = updated_capacity_route1;
                    updated_routes_capacities[route2] = updated_capacity_route2;
                    swap_cities(updated_routes, route1, route2, idx1, idx2);
                    did_exchange = 1;
                }
            }
        }
    }
}


// DELETE AND INSERT
void move(vector<vector<int>> &updated_routes, vector<int> &updated_routes_capacities, instance data_inst, int route_del, int route_ins, int idx_del, int idx_ins) {
    int moved_city = updated_routes[route_del][idx_del];
    
    updated_routes[route_del].erase(updated_routes[route_del].begin() + idx_del);
    if (updated_routes[route_del].size() == 0) {
      updated_routes.erase(updated_routes.begin() + route_del);
      updated_routes_capacities.erase(updated_routes_capacities.begin() + route_del);
    }
    updated_routes[route_ins].insert(updated_routes[route_ins].begin() + idx_ins, moved_city);
    
    // capacities are not updated if both deleted and inserted are in same route
    if (route_del != route_ins) {
        int capacity_route_ins = updated_routes_capacities[route_ins] + data_inst.demands[moved_city];
        int capacity_route_del = updated_routes_capacities[route_del] - data_inst.demands[moved_city];
        updated_routes_capacities[route_del] = capacity_route_del;
        updated_routes_capacities[route_ins] = capacity_route_ins;
    }
}

void delete_and_insert(vector<vector<int>> &updated_routes, vector<int> &updated_routes_capacities, instance data_inst) {
    int did_exchange = 0;
    while (!did_exchange) {
        // randomly select index to delete and idx to insert - do not allow index 0
        int route_del = rand() % updated_routes.size();
        int route_ins = rand() % updated_routes.size();
        int idx_del = (rand() % (updated_routes[route_del].size() - 1)) + 1;
        int idx_ins = (rand() % (updated_routes[route_ins].size() - 1)) + 1;
        
        // find the cities in those indexes
        int city_del = updated_routes[route_del][idx_del];
        
        if (route_del != route_ins || idx_del != idx_ins) {
            // delete and add within the same route - no need to check for capacity constraints
            if (route_del == route_ins) {
                move(updated_routes, updated_routes_capacities, data_inst, route_del, route_ins, idx_del, idx_ins);
                did_exchange = 1;
            }
            // add to different routes - need to check if capacity is exceeded
            else {
                int capacity_route_ins = updated_routes_capacities[route_ins] + data_inst.demands[city_del];
                
                if (capacity_route_ins < data_inst.uniform_vehicle_capacity) {
                    move(updated_routes, updated_routes_capacities, data_inst, route_del, route_ins, idx_del, idx_ins);
                    did_exchange = 1;
                }
            }
        }
    }
}

// REVERSE
vector<int> reverse_aux(vector<int> route, int i, int k) {
    vector<int> new_route(route);
    
    int it = 0;
    while (it <= (k-i)/2) {
        new_route[i+it] = route[k-it];
        new_route[k-it] = route[i+it];
        it++;
    }
    return new_route;
}

void reverse_route(vector<vector<int>> &updated_routes, vector<int> &updated_routes_capacities, instance data_inst) {
    int idx = rand() % updated_routes.size();
    
    vector<int> route(updated_routes[idx]);

    int best_distance = route_cost(route, data_inst);
    
    for (int i = 1; i < route.size() - 1; i++) {
        for (int k = i + 1; k < route.size(); k++) {
            vector<int> new_route = reverse_aux(route, i, k);
            int new_distance = route_cost(new_route, data_inst);
            if (new_distance < best_distance) {
                route = new_route;
            }
        }
    }
    
    updated_routes[idx] = route;
}

neighborhood_generator::neighborhood_generator(instance inst) { 
  data_inst = inst;
}

neighborhood_generator::neighborhood_generator() { }


/**
 * CVRP can be divided into:
 * Allocation of customers (inter-route optimization)
 * Optimization of the route itself (intra-route optimization)
 */

/**
 * Neighborhoods updates
 *
 * - Exchange: randomly swap two nodes with each other
 * - Delete and insert: delete an arbitrary node and insert it to another position
 * - Reverse: reverse visitation order in a part of a route
 */
void neighborhood_generator::update_solution(vector<vector<int>> &updated_routes, vector<int> &updated_route_capacities) {
    int type = rand() % 2;
    type = 1;
    switch(type) {
        case 0:
            exchange(updated_routes, updated_route_capacities, data_inst);
        case 1:
            delete_and_insert(updated_routes, updated_route_capacities, data_inst);
        case 2:
            reverse_route(updated_routes, updated_route_capacities, data_inst);
    }
}
