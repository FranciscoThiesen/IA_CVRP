#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <map>

using namespace std;

vector< string > split_line( string line ) {
    stringstream sl(line);
    istream_iterator<string> begin(sl);
    istream_iterator<string> end;
    vector<string> words(begin, end);
    return words;
}

struct cvrp_instance {
    
    vector< pair<int, int> > points;
    vector< vector<int> > adjacency_matrix;
    vector< int > demands;
    
    string instance_file;
    string instance_name;
    
    int dimension;
    int depot_index;
    int uniform_vehicle_capacity;
    
    inline int euclidean_distance( const pair<int, int>& a, const pair<int, int>& b) {
        int dx = (a.first - b.first) * (a.first - b.first);
        int dy = (a.second - b.second) * (a.second - b.second);
        return (int) ceil(sqrt( dx + dy ) );
    }
    
    // Function for reading instance file in the format specified at (vrp.atd-lab.inf.puc-rio.br/media/com_vrp)
    void load_instance(string filename ) {
        ifstream in(filename);
        string line;
        vector< vector<string> > file_lines;
        
        while( getline(in, line) ) {
            auto x = split_line(line);
            file_lines.emplace_back( x );
        }
        in.close();
        
        // following dataset convention
        instance_name = file_lines[0][2];
        instance_file = filename;
        dimension = stoi(file_lines[3][2]);
        uniform_vehicle_capacity = stoi( file_lines[5][2] );
        
        const int graphdata_start = 7;
        
        for(int node = 0; node < dimension; ++node) {
            int x = stoi( file_lines[graphdata_start + node][1] );
            int y = stoi( file_lines[graphdata_start + node][2] );
            points.emplace_back(x , y);
        }
        
        const int demand_start = graphdata_start + dimension + 1;
        
        for(int node = 0; node < dimension; ++node) {
            int demand = stoi( file_lines[demand_start + node][1] );
            demands.emplace_back( demand );
        }
        
        depot_index = stoi(file_lines[demand_start + dimension + 1][0]) - 1; // our information is 0-indexed, as it SHOULD ALWAYS BE
        
    }
    
    void adjacency_matrix_initialization() {
        adjacency_matrix.assign( dimension, vector<int>(dimension, 0) );
        for(int i = 0; i < dimension; ++i) {
            for(int j = i + 1; j < dimension; ++j) {
                adjacency_matrix[i][j] = adjacency_matrix[j][i] = euclidean_distance( points[i], points[j] );
            }
        }
    }
    
    void print_graph() {
        for(int i = 0; i < dimension; ++i)
            cout << i << " " << points[i].first << " " << points[i].second << " " << demands[i] << endl;
    }
    
    cvrp_instance( string _instance_file ) {
        instance_file = _instance_file;
        load_instance( instance_file );
        adjacency_matrix_initialization();
    }
    
    cvrp_instance() {}
};

struct simulated_annealing {
    cvrp_instance instance;
    vector<vector<int>> cur_routes; // vector containing which node belongs to which routes (the end of the route is delimited by zero)
    vector<int> cur_routes_capacities; // contains capacity for every route
    int cur_route_cost;
    
    vector<vector<int>> best_routes;
    int best_route_cost;
    
    simulated_annealing(cvrp_instance ins) {
        instance = ins;
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
                cout << visited_city << " " << instance.points[visited_city].first << " " << instance.points[visited_city].second << " " << instance.demands[visited_city] << endl;
                route_capacity += instance.demands[visited_city];
                
                if (visited_city == instance.depot_index) {
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
    
    // Returns total route capacity in log(n) time
    int route_capacity(vector<vector<int>> routes, int route_idx) {
        vector<int> route = routes[route_idx];
        int capacity = 0;
        for (int i = 0; i < route.size(); i++) {
            capacity += instance.demands[route[i]];
        }
        return capacity;
    }
    
    int route_cost(vector<int> route) {
        int cost = 0;
        for (int i = 0; i < route.size() - 1; i++) {
            int dist = instance.adjacency_matrix[route[i]][route[i+1]];
            cost += dist;
        }
        int dist_origin = instance.adjacency_matrix[route[route.size() - 1]][instance.depot_index];
        cost += dist_origin;
        return cost;
    }
    
    int solution_cost(vector<vector<int>> routes) {
        int cost = 0;
        for (int r = 0; r < routes.size() - 1; r++) {
            vector<int> route = routes[r];
            cost += route_cost(route);
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
        vector<int> city_visited_status(instance.dimension); // by default, initializes to 0
        int visited_cities = 1;
        city_visited_status[instance.depot_index] = 1;
        while (visited_cities < instance.dimension) {
            int current_capacity = 0;
            vector<int> route;
            route.emplace_back(instance.depot_index);
            
            for (int v = 0; v < instance.dimension; v++) {
                if (city_visited_status[v] == 0 &&
                    current_capacity + instance.demands[v] < instance.uniform_vehicle_capacity) {
                    route.emplace_back(v);
                    current_capacity += instance.demands[v];
                    city_visited_status[v] = 1;
                    visited_cities++;
                }
            }
            cur_routes.emplace_back(route);
            cur_routes_capacities.emplace_back(current_capacity);
        }
    }
    
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
    void update_solution(vector<vector<int>> &updated_routes, vector<int> &updated_route_capacities) {
        int type = rand() % 2;
        switch(type) {
            case 0:
                exchange(updated_routes, updated_route_capacities);
            case 1:
                delete_and_insert(updated_routes, updated_route_capacities);
            case 2:
                reverse_route(updated_routes, updated_route_capacities);
        }
        
    }
    
    
    // EXCHANGE
    /**
     * Swaps the depot with a city. Creates a new route starting
     * in the depot (where the city was - divides an existing route)
     * and updates the capacity of the route that was extended
     * (depot was substituted by a city)
     */
    void swap_cities(vector<vector<int>> &updated_routes, int route1, int route2, int idx1, int idx2) {
        int temp = updated_routes[route1][idx1];
        updated_routes[route1][idx1] = updated_routes[route2][idx2];
        updated_routes[route2][idx2] = temp;
    }
    
    void exchange(vector<vector<int>> &updated_routes, vector<int> &updated_routes_capacities) {
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
                    
                    int updated_capacity_route1 = capacity_route1 - instance.demands[city_idx1] + instance.demands[city_idx2];
                    int updated_capacity_route2 = capacity_route2 - instance.demands[city_idx2] + instance.demands[city_idx1];
                    
                    if (updated_capacity_route1 < instance.uniform_vehicle_capacity &&
                        updated_capacity_route2 < instance.uniform_vehicle_capacity) {
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
    void move(vector<vector<int>> &updated_routes, vector<int> &updated_routes_capacities, int route_del, int route_ins, int idx_del, int idx_ins) {
        int moved_city = updated_routes[route_del][idx_del];
        
        updated_routes[route_del].erase(updated_routes[route_del].begin() + idx_del);
        updated_routes[route_ins].insert(updated_routes[route_ins].begin() + idx_ins, moved_city);
        
        // capacities are not updated if both deleted and inserted are in same route
        if (route_del != route_ins) {
            int capacity_route_ins = updated_routes_capacities[route_ins] + instance.demands[moved_city];
            int capacity_route_del = updated_routes_capacities[route_del] - instance.demands[moved_city];
            updated_routes_capacities[route_del] = capacity_route_del;
            updated_routes_capacities[route_ins] = capacity_route_ins;
        }
    }
    
    void delete_and_insert(vector<vector<int>> &updated_routes, vector<int> &updated_routes_capacities) {
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
                    move(updated_routes, updated_routes_capacities, route_del, route_ins, idx_del, idx_ins);
                    did_exchange = 1;
                }
                // add to different routes - need to check if capacity is exceeded
                else {
                    int capacity_route_ins = updated_routes_capacities[route_ins] + instance.demands[city_del];
                    
                    if (capacity_route_ins < instance.uniform_vehicle_capacity) {
                        move(updated_routes, updated_routes_capacities, route_del, route_ins, idx_del, idx_ins);
                        did_exchange = 1;
                    }
                }
            }
            
            if (did_exchange) {
                check_routes_data(updated_routes, updated_routes_capacities);
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
    
    void reverse_route(vector<vector<int>> &updated_routes, vector<int> &updated_routes_capacities) {
        int idx = rand() % updated_routes.size();
        
        vector<int> route(updated_routes[idx]);
    
        int best_distance = route_cost(route);
        
        for (int i = 1; i < route.size() - 1; i++) {
            for (int k = i + 1; k < route.size(); k++) {
                vector<int> new_route = reverse_aux(route, i, k);
                int new_distance = route_cost(new_route);
                if (new_distance < best_distance) {
                    route = new_route;
                }
            }
        }
        
        updated_routes[idx] = route;
    }
    
    /*
    * Simulated Annealing
    */
    int should_update(float cost_diff, float temp) {
        float prob = exp(-cost_diff / temp) * 100;
        return (rand() % 100 < prob);
    }
    
    vector<vector<int>> annealing_CVRP(float initial_temperature, float temp_factor) {
//        const float initial_temperature = 5000;
//        const float temp_factor = 0.9;  // temperature reduction multiplier
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
            
            update_solution(updated_routes, updated_route_capacities);
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
        vector<float> temp_factors = {0.85, 0.9, 0.95};
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
        printf("Best temp: %.2f, best factor: %.2f\n", best_temp, best_factor);
    }
    
    void check_routes_data(vector<vector<int>> routes, vector<int> route_capacities) {
        for (int r = 0; r < routes.size(); r++) {
            int capacity = 0;
            for (int i = 0; i < routes[r].size(); i++) {
                capacity += instance.demands[routes[r][i]];
            }
            if (route_capacities[r] != capacity) {
                printf("route idx: %d, expected: %d, actual: %d\n", r, capacity, route_capacities[r]);
            }
        }
    }
};

int main()
{
    cvrp_instance x("instances/X-n101-k25.vrp");
    simulated_annealing annealing(x);
}
