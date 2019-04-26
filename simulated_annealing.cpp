#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdio>
#include "data_loader.h"
#include "neighborhood_generator.h"

struct simulated_annealing {
    instance data_inst;
    neighborhood_generator n_generator;
    vector<vector<int>> cur_routes; // vector containing which node belongs to which routes (the end of the route is delimited by zero)
    vector<int> cur_routes_capacities; // contains capacity for every route
    int cur_route_cost;
    
    vector<vector<int>> best_routes;
    int best_route_cost;
    
    simulated_annealing(instance ins) {
        data_inst = ins;
        n_generator = neighborhood_generator(ins);
        test_constants();
    }
    
    /**
     * Auxiliary functions
     */
    void print_solution(vector<vector<int>> routes) {
        int route_capacity = 0;
        cout << "Route" << endl;
        for (int r = 0; r < routes.size(); r++) {
            vector<int> route = routes[r];
            for (int i = 0; i < route.size(); i++ ) {
                int visited_city = route[i];
                cout << visited_city << " " << data_inst.points[visited_city].first << " " << data_inst.points[visited_city].second << " " << data_inst.demands[visited_city] << endl;
                route_capacity += data_inst.demands[visited_city];
                if (visited_city == data_inst.depot_index) {
                    cout << "Route capacity: " << route_capacity << endl << endl;
                    route_capacity = 0;
                    cout << "Route" << endl;
                }
            }
        }
    }
    
    void print_vec(vector<int > v) {
        for (int i = 0; i < v.size(); i++) {
            cout << v[i] << ", ";
        }
        cout << endl;
    }
  
    int route_capacity(vector<vector<int>>& routes, int route_idx) {
        vector<int> route = routes[route_idx];
        int capacity = 0;
        for (int i = 0; i < route.size(); i++) {
            capacity += data_inst.demands[route[i]];
        }
        return capacity;
    }
    
    int route_cost(vector<int> route) {
        int cost = 0;
        for (int i = 0; i < route.size() - 1; i++) {
            int dist = data_inst.adjacency_matrix[route[i]][route[i+1]];
            cost += dist;
        }
        int dist_origin = data_inst.adjacency_matrix[route[route.size() - 1]][data_inst.depot_index];
        cost += dist_origin;
        return cost;
    }
    
    int solution_cost(vector<vector<int>> routes) {
        int cost = 0;
        for (int r = 0; r < routes.size(); r++) {
            cost += route_cost(routes[r]);
        }
        return cost;
    }
    
    /**
     * Generates an initial solution based on a greedy
     * algorithm that creates non-optimal feasible routes
     * of visitation based on what fits first.
     */
    void initial_solution_greedy() {
        cur_routes.clear();
        cur_routes_capacities.clear();
        vector<int> city_visited_status(data_inst.dimension); // by default, initializes to 0
        int visited_cities = 1;
        city_visited_status[data_inst.depot_index] = 1;
        while (visited_cities < data_inst.dimension) {
            int current_capacity = 0;
            vector<int> route;
            route.emplace_back(data_inst.depot_index);
            
            for (int v = 0; v < data_inst.dimension; v++) {
                if (city_visited_status[v] == 0 &&
                    current_capacity + data_inst.demands[v] < data_inst.uniform_vehicle_capacity) {
                    route.emplace_back(v);
                    current_capacity += data_inst.demands[v];
                    city_visited_status[v] = 1;
                    visited_cities++;
                }
            }
            cur_routes.emplace_back(route);
            cur_routes_capacities.emplace_back(current_capacity);
        }
    }
    
    /*
    * Simulated Annealing
    */
    int should_update(float cost_diff, float temp) {
        float prob = exp(-cost_diff / temp) * 100;
        return (rand() % 100 < prob);
    }
    
    vector<vector<int>> annealing_CVRP(float initial_temperature, float temp_factor) {
        const float cutoff_time = 5; // iterations for a given temperature until the next update
        const float max_time_improvement = 5000;
        const float min_temp = 0.0001;
        
        int temp_time = 0;
        int time_since_improvement = 0;
        float temperature = initial_temperature;
        
        initial_solution_greedy();
        cur_route_cost = solution_cost(cur_routes);
        
        best_routes = cur_routes;
        best_route_cost = solution_cost(best_routes);
        
        while (temperature > min_temp || time_since_improvement < max_time_improvement) {
            time_since_improvement++;
            vector<vector<int>> updated_routes(cur_routes);
            vector<int> updated_route_capacities(cur_routes_capacities);
            n_generator.update_solution(updated_routes, updated_route_capacities);
            float new_cost = solution_cost(updated_routes);
            float cost_diff = new_cost - cur_route_cost;
            if (cost_diff < 0) { // update improved solution
                time_since_improvement = 0;
                cur_routes = updated_routes;
                cur_routes_capacities = updated_route_capacities;
                cur_route_cost = new_cost;
                cout << "Cost updated: " << cur_route_cost << "-> " << new_cost << endl;
                if (new_cost < best_route_cost) {
                    best_routes.assign(updated_routes.begin(), updated_routes.end());
                    best_route_cost = new_cost;
                }
            }
            else if (cost_diff != 0 && should_update(cost_diff, temperature)) {
                cout << "Cost updated randomly: " << cur_route_cost << "-> " << new_cost << endl;
                
                cur_routes = updated_routes;
                cur_routes_capacities = updated_route_capacities;
                cur_route_cost = new_cost;
            }
            temp_time++;
            if (temp_time == cutoff_time) {
                temp_time = 0;
                temperature *= temp_factor;
            }
        }
//        print_solution(best_routes);
        return best_routes;
    }
    
    void test_constants() {
        vector<float> initial_temperatures = {10000, 9000, 8000, 7000, 6000, 5000, 4000, 3000, 2000, 1000, 500};
        vector<float> temp_factors = {0.9};
        float best_params_cost = 10e5;
        float best_temp, best_factor;
        for (int t = 0; t < initial_temperatures.size(); t++) {
            for (int f = 0; f < temp_factors.size(); f++) {
                printf("initial temp: %.1f,  factor: %.2f", initial_temperatures[t], temp_factors[f]);
                annealing_CVRP(initial_temperatures[t], temp_factors[f]);
                if (best_route_cost < best_params_cost) {
                    best_params_cost = best_route_cost;
                    best_temp = initial_temperatures[t];
                    best_factor = temp_factors[f];
                }
            }
        }
        printf("Best temp: %.2f, best factor: %.2f, best cost: %.2f\n", best_temp, best_factor, best_params_cost);
    }
    
    void check_routes_data(vector<vector<int>> routes, vector<int> route_capacities) {
        for (int r = 0; r < routes.size(); r++) {
            int capacity = 0;
            for (int i = 0; i < routes[r].size(); i++) {
                capacity += data_inst.demands[routes[r][i]];
            }
            if (route_capacities[r] != capacity) {
                printf("route idx: %d, expected: %d, actual: %d\n", r, capacity, route_capacities[r]);
            }
        }
    }
};

int main()
{
    instance x("instances/X-n101-k25.vrp");
    simulated_annealing annealing_CVRP(x);
}
