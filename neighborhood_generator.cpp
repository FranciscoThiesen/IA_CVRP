#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdio>
#include "data_loader.h"
#include "neighborhood_generator.h"
#include <random>

using namespace std;

int route_cost(vector<int> route, instance data_inst) {
    int cost = 0;
    for (int i = 0; i < (int)route.size() - 1; i++) {
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

bool apply_best_exchange(vector< vector<int> >& updated_routes, vector< int >& updated_route_capacities, instance data_inst)
{
    int total_routes = (int) updated_routes.size();
    int best_benefit = -1;
    int best_first_route = -1;
    int best_first_index = -1;
    int best_second_route = -1;
    int best_second_index = -1;

    for(int first_route = 0; first_route < total_routes; ++first_route) {
        int fst_sz = (int) updated_routes[first_route].size();
        for(int second_route = first_route; second_route < total_routes; ++second_route) {
            int snd_sz = (int) updated_routes[second_route].size();
            for(int first_index = 1; first_index < fst_sz; ++first_index) {
                for(int second_index = 1; second_index < snd_sz; ++second_index) {
                    int F = updated_routes[first_route][first_index];
                    int S = updated_routes[second_route][second_index];
                    int cap_fst = updated_route_capacities[first_route], cap_snd = updated_route_capacities[second_route];
                    int upd_cap_fst = cap_fst - data_inst.demands[F] + data_inst.demands[S];
                    int upd_cap_snd = cap_snd - data_inst.demands[S] + data_inst.demands[F];
                    if ( max(upd_cap_fst, upd_cap_snd) <= data_inst.uniform_vehicle_capacity) {
                        int prev_fst = updated_routes[first_route][first_index - 1];
                        int next_fst = ( first_index == fst_sz - 1 ? data_inst.depot_index : updated_routes[first_route][first_index + 1] );
                        int prev_snd = updated_routes[second_route][second_index - 1];
                        int next_snd = ( second_index == snd_sz - 1? data_inst.depot_index : updated_routes[second_route][second_index + 1]);
                        int gain = data_inst.adjacency_matrix[prev_fst][F] + data_inst.adjacency_matrix[F][next_fst];
                        gain += data_inst.adjacency_matrix[prev_snd][S] + data_inst.adjacency_matrix[S][next_snd];
                        gain -= ( data_inst.adjacency_matrix[prev_fst][S] + data_inst.adjacency_matrix[S][next_fst]);
                        gain -= ( data_inst.adjacency_matrix[prev_snd][F] + data_inst.adjacency_matrix[F][next_snd]);

                        if( gain > best_benefit ) {
                            best_first_route = first_route; best_first_index = first_index;
                            best_second_route = second_route; best_second_index = second_index; 
                            best_benefit = gain;
                        }
                    }
                }
            }
        }
    }
    if( best_benefit <= 0) return false;
    else {
        swap_cities( updated_routes, best_first_route, best_second_route, best_first_index, best_second_index );
        return true;
    }
    return false;
}
// DELETE AND INSERT
void move(vector<vector<int>> &updated_routes, vector<int> &updated_routes_capacities, instance data_inst, int route_del, int route_ins, int idx_del, int idx_ins) {
    int moved_city = updated_routes[route_del][idx_del];

    updated_routes[route_del].erase(updated_routes[route_del].begin() + idx_del);
    updated_routes[route_ins].insert(updated_routes[route_ins].begin() + idx_ins, moved_city);
    
    // capacities are not updated if both deleted and inserted are in same route
    if (route_del != route_ins) {
        int capacity_route_ins = updated_routes_capacities[route_ins] + data_inst.demands[moved_city];
        int capacity_route_del = updated_routes_capacities[route_del] - data_inst.demands[moved_city];
        updated_routes_capacities[route_ins] = capacity_route_ins;        
        if (updated_routes[route_del].size() == 1) {
          updated_routes.erase(updated_routes.begin() + route_del);
          updated_routes_capacities.erase(updated_routes_capacities.begin() + route_del);
        } else {
            updated_routes_capacities[route_del] = capacity_route_del;
        }
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
    
    for (int i = 1; i < (int)route.size() - 1; i++) {
        for (int k = i + 1; k < (int)route.size(); k++) {
            vector<int> new_route = reverse_aux(route, i, k);
            int new_distance = route_cost(new_route, data_inst);
            if (new_distance < best_distance) {
                route = new_route;
            }
        }
    }
    
    updated_routes[idx] = route;
}

bool apply_best_delete_and_insert( vector< vector<int> >& updated_routes, vector<int>& updated_routes_capacities, instance data_inst)
{
    // Remember that the first element from every route is the depot
    int best_benefit = -1;
    int best_deleted_route = -1;
    int best_deleted_index = -1;
    int best_inserted_route = -1;
    int best_inserted_index = -1;
    int total_routes = (int) updated_routes.size();
    for(int delete_route = 0; delete_route < total_routes; ++delete_route) {
        int sz_del = (int) updated_routes[delete_route].size();
        for(int insert_route = 0; insert_route < total_routes; ++insert_route) {
            int sz_ins = (int) updated_routes[insert_route].size();
            for(int delete_index = 1; delete_index < sz_del; ++delete_index) {
                for(int insert_index = 1; insert_index < sz_ins; ++insert_index) {
                    if(updated_routes_capacities[insert_route] + data_inst.demands[ updated_routes[delete_route][delete_index] ] <= data_inst.uniform_vehicle_capacity ) {
                        int cur_deleted = updated_routes[delete_route][delete_index - 1];
                        int prev_deleted = updated_routes[delete_route][delete_index - 1];
                        int next_deleted = ( delete_index == sz_del - 1 ? data_inst.depot_index : updated_routes[delete_route][delete_index + 1]);
                        int savings = data_inst.adjacency_matrix[ prev_deleted ][ cur_deleted ];
                        savings += data_inst.adjacency_matrix[ cur_deleted ][ next_deleted ];
                        savings -= data_inst.adjacency_matrix[ prev_deleted ][ next_deleted ];

                        int prev_insert = updated_routes[insert_route][insert_index - 1];
                        int next_insert = updated_routes[insert_route][insert_index];

                        savings += data_inst.adjacency_matrix[prev_insert][next_insert];
                        savings -= data_inst.adjacency_matrix[prev_insert][cur_deleted];
                        savings -= data_inst.adjacency_matrix[cur_deleted][next_insert];
                        
                        if( savings > best_benefit )
                        {
                            best_benefit = savings;
                            best_deleted_route = delete_route;
                            best_deleted_index = delete_index;
                            best_inserted_route = insert_route;
                            best_inserted_index = insert_index;
                        }
                    }
                }
            }
        }
    }
    if( best_benefit <= 0) return false;
    else {
        if( best_deleted_route != best_inserted_route || best_deleted_index != best_inserted_index) {
            if( best_deleted_route == best_inserted_route ) move(updated_routes, updated_routes_capacities, data_inst, best_deleted_route, best_inserted_route, best_deleted_index, best_inserted_index);
            else move(updated_routes, updated_routes_capacities, data_inst, best_deleted_route, best_inserted_route, best_deleted_index, best_inserted_index);
            return true;
        }
    }
    return true; // just to avoid warning
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
    int type = rand() % 3;
    switch(type) {
        case 0:
            exchange(updated_routes, updated_route_capacities, data_inst);
        case 1:
            delete_and_insert(updated_routes, updated_route_capacities, data_inst);
        case 2:
            reverse_route(updated_routes, updated_route_capacities, data_inst);
    }
}

bool neighborhood_generator::update_solution_best_improvement( vector< vector<int> >& updated_routes, vector<int>& updated_route_capacities) {
    int nei = rand() % 2;
    if(nei == 0) {
        return apply_best_exchange(updated_routes, updated_route_capacities, data_inst);    
    }
    else {
        return apply_best_delete_and_insert(updated_routes, updated_route_capacities, data_inst);
    }
    return false;
}
void neighborhood_generator::update_solution_deterministic(vector<vector<int>>& updated_routes, vector<int>& updated_route_capacities, int n_type) 
{
    switch(n_type) {
        case 0:
            exchange(updated_routes, updated_route_capacities, data_inst);
        case 1:
            delete_and_insert(updated_routes, updated_route_capacities, data_inst);
        case 2:
            reverse_route(updated_routes, updated_route_capacities, data_inst);
    }
}

void neighborhood_generator::set_seed(int s) {
    seed = s;
    srand(seed);
}

void neighborhood_generator::update_solution_custom(vector< vector<int> >& updated_routes, vector<int>& updated_route_capacities, vector<int>& neighborhood_indices)
{
    int distinct_neighborhoods = (int) neighborhood_indices.size();
    int gen = ( rand() % distinct_neighborhoods );
    int v = neighborhood_indices[gen];
    if( v == 0) exchange( updated_routes, updated_route_capacities, data_inst );
    else if( v == 1 ) delete_and_insert( updated_routes, updated_route_capacities, data_inst);
    else reverse_route( updated_routes, updated_route_capacities, data_inst);
}
