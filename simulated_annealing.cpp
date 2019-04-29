#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <map>
#include "data_loader.h"
#include "neighborhood_generator.h"
#include "time_lib.h"

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
        for (int r = 0; r < (int) routes.size(); r++) {
            vector<int> route = routes[r];
            for (int i = 0; i < (int) route.size(); i++ ) {
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
        for (int i = 0; i < (int) v.size(); i++) {
            cout << v[i] << ", ";
        }
        cout << endl;
    }
  
    int route_capacity(vector<vector<int>>& routes, int route_idx) {
        vector<int> route = routes[route_idx];
        int capacity = 0;
        for (int i = 0; i < (int) route.size(); i++) {
            capacity += data_inst.demands[route[i]];
        }
        return capacity;
    }
    
    int route_cost(vector<int> route) {
        int cost = 0;
        for (int i = 0; i < (int) route.size() - 1; i++) {
            int dist = data_inst.adjacency_matrix[route[i]][route[i+1]];
            cost += dist;
        }
        int dist_origin = data_inst.adjacency_matrix[route[route.size() - 1]][data_inst.depot_index];
        cost += dist_origin;
        return cost;
    }
    
    int solution_cost(vector<vector<int>> routes) {
        int cost = 0;
        for (int r = 0; r < (int) routes.size(); r++) {
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
    
    void smart_greedy() {
        cur_routes.clear();
        cur_routes_capacities.clear();
        
        vector< vector<int> > routes(1, vector<int>(1, data_inst.depot_index));
        vector< int > route_demand(1, 0);
        vector< tuple<int, int, int> > points;
        pair<int, int> center = data_inst.points[data_inst.depot_index];
        for(int p = 0; p < data_inst.dimension; ++p)
        {
            if( p == data_inst.depot_index ) continue;
            points.emplace_back( data_inst.points[p].first, data_inst.points[p].second, p );
        }
        // Radial sort
        sort( points.begin(), points.end(), [&] (tuple<int, int, int>& a, tuple<int, int, int>& b)
             {
                 pair<int, int> pa = make_pair(get<0>(a) - center.first, get<1>(a) - center.second);
                 pair<int, int> pb = make_pair(get<0>(b) - center.first, get<1>(b) - center.second);
                 int cross_product = pa.first * pb.second - pa.second * pb.first;
                 if( cross_product != 0 ) return (cross_product > 0);
                 pa = make_pair(pa.first * pa.first, pa.second * pa.second );
                 pb = make_pair(pb.first * pb.first, pb.second * pb.second );
                 int len_a = pa.first + pa.second;
                 int len_b = pb.first + pb.second;
                 
                 return len_a < len_b;
             });
        
        int current_route = 0;
        for(const auto& P : points)
        {
            int point_index = get<2>(P);
            if( data_inst.demands[point_index] + route_demand[current_route] <= data_inst.uniform_vehicle_capacity )
            {
                route_demand[current_route] += data_inst.demands[point_index];
                routes[current_route].push_back( point_index );
            }
            else
            {
                current_route++;
                route_demand.push_back(0);
                route_demand[current_route] += data_inst.demands[point_index];
                routes.push_back( vector<int>(1, data_inst.depot_index) );
                routes.back().push_back( point_index );
                
            }
        }
        
        cur_routes = routes;
        cur_routes_capacities = route_demand;
        cur_route_cost = solution_cost(cur_routes);
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
        const float max_time_improvement = 10000;
        
        int temp_time = 0;
        int time_since_improvement = 0;
        float temperature = initial_temperature;
        
        smart_greedy();
        cur_route_cost = solution_cost(cur_routes);
        
        best_routes = cur_routes;
        best_route_cost = solution_cost(best_routes);
        
        while (time_since_improvement < max_time_improvement) {
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
                if (new_cost < best_route_cost) {
                    best_routes.assign(updated_routes.begin(), updated_routes.end());
                    best_route_cost = new_cost;
                }
            }
            else if (cost_diff != 0 && should_update(cost_diff, temperature)) {
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
        vector<int> initial_temperatures = {10000, 9000, 8000, 7000, 6000, 5000, 4000, 3000, 2000, 1000, 500};
        vector<float> temp_factors = {0.85, 0.9, 0.95};
        int best_params_cost = 10e5;
        //int best_temp; float best_factor;
        map<pair<int, float>, pair<int, long double> > param_costs;
        string csv_name = data_inst.instance_name;
        int instance_BKS = 0;
        if( csv_name == "X-n101-k25" ) instance_BKS = 27591;
        else if( csv_name == "X-n110-k13") instance_BKS = 14971;
        else if( csv_name == "X-n115-k10") instance_BKS = 12747;
        else instance_BKS = 19565;
        cout << "BKS = " << instance_BKS << endl;
        csv_name += ".csv"; 
        csv_name = "simulated_annealing_results/" + csv_name; 
        ofstream out(csv_name);
        for (int t = 0; t < (int) initial_temperatures.size(); t++) {
            for (int f = 0; f < (int) temp_factors.size(); f++) {
                cout << "rodando t = " << t << " f = " << f << endl;
                clock_t start = get_time();
                annealing_CVRP(initial_temperatures[t], temp_factors[f]);
                clock_t end = get_time();
                long double duration = time_in_ms(start, end); 
                param_costs[make_pair(initial_temperatures[t], temp_factors[f])] = make_pair(best_route_cost, duration);
                if (best_route_cost < best_params_cost) {
                    best_params_cost = best_route_cost;
                    //best_temp = initial_temperatures[t];
                    //best_factor = temp_factors[f];
                }
            }
        }
        
        out << "Temperatura inicial,Fator de temperatura,Tempo (ms),Solucao,BKS,Approximation Ratio" << endl;
        for(const auto& entry : param_costs) {
            out << entry.first.first << "," << entry.first.second << "," << entry.second.second << "," << entry.second.first << "," << instance_BKS << "," << 1.0 * entry.second.first / instance_BKS << endl;
        }
        out.close();
    }
    
    void check_routes_data(vector<vector<int>> routes, vector<int> route_capacities) {
        for (int r = 0; r < (int) routes.size(); r++) {
            int capacity = 0;
            for (int i = 0; i < (int) routes[r].size(); i++) {
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
        vector<string> instances = {"instances/X-n101-k25.vrp", "instances/X-n110-k13.vrp", "instances/X-n115-k10.vrp", "instances/X-n204-k19.vrp"};
        for (const string& file: instances) {
          instance x(file);
          simulated_annealing annealing_CVRP(x);
        }
}
