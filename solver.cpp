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
    vector<int> cur_routes;
    map<int, int> cur_routes_data; // contains starting index and capacity
    int cur_route_cost;
    
    vector<int> best_routes;
    int best_route_cost;
    
    simulated_annealing(cvrp_instance ins) {
        instance = ins;
        annealing_CVRP();
    }
    
    /**
     * Auxiliary functions
     */
    void print_solution(vector<int > routes) {
        int route_capacity = 0;
        cout << "Route" << endl;
        for (int v = 0; v < routes.size(); v++) {
            int visited_city = routes[v];
            cout << visited_city << " " << instance.points[visited_city].first << " " << instance.points[visited_city].second << " " << instance.demands[v] << endl;
            route_capacity += instance.demands[visited_city];
            
            if (visited_city == instance.depot_index) {
                cout << "Route capacity: " << route_capacity << endl << endl;
                route_capacity = 0;
                cout << "Route" << endl;
            }
        }
    }
    
    void print_vec(vector<int > v) {
        for (int i = 0; i < v.size(); i++) {
            cout << v[i] << ", ";
        }
        cout << endl;
    }
    
    pair<int, int> route_bounds(map<int, int> routes_data, int idx) {
        map<int, int>::iterator next_route_start = routes_data.lower_bound(idx);
        pair<int, int> bounds;
        if (next_route_start->first != 0) {
            map<int, int>::iterator route_start = prev(next_route_start);
            bounds = make_pair(route_start->first, next_route_start->first);
        } else {
            bounds = make_pair(0, 0);
        }
        return bounds;
    }
    
    int route_capacity_for_bounds(vector<int> routes, pair<int, int> bounds) {
        int capacity = 0;
        for (int i = bounds.first; i < bounds.second; i++) {
            capacity += instance.demands[routes[i]];
        }
        return capacity;
    }
    int route_capacity(vector<int> routes, map<int, int> routes_data, int idx) {
        pair<int, int> bounds = route_bounds(routes_data, idx);
        
        return route_capacity_for_bounds(routes, bounds);
    }
    
    int solution_cost(vector<int> routes) {
        int cost = 0;
        for (int v = 0; v < routes.size() - 1; v++) {
            int dist = instance.adjacency_matrix[routes[v]][routes[v+1]];
            cost += dist;
        }
        return cost;
    }
    
    /**
     * Generates an initial solution based on a greedy
     * algorithm that creates non-optimal feasible routes
     * of visitation based on what fits first.
     */
    void initial_solution_greedy() {
        vector<int> city_visited_status(instance.dimension); // by default, initializes to 0
        int total_routes = 0;
        int visited_cities = 0;
        int start_index = 0;
        while (visited_cities < instance.dimension) {
            int current_capacity = 0;
            for (int v = 0; v < instance.dimension; v++) {
                if (city_visited_status[v] == 0 &&
                    current_capacity + instance.demands[v] < instance.uniform_vehicle_capacity) {
                    cur_routes.emplace_back(v);
                    current_capacity += instance.demands[v];
                    city_visited_status[v] = 1;
                    visited_cities++;
                }
            }
            cur_routes_data[start_index] = current_capacity;
            start_index = cur_routes.size();
            cur_routes.emplace_back(instance.depot_index);
            total_routes++;
        }
    }
    
    
    /**
     * Transformation rules for generating neighbors
     *
     * - One to one exchange: randomly swap two nodes with each other
     * - Delete and insert: delete an arbitrary node and insert it to another position
     * - Partial reversal: reverse visitation order in a part of a route
     */
    void update_solution(vector<int> *updated_routes, map<int, int> *updated_route_data) {
        one_to_one_exchange(updated_routes, updated_route_data);
    }
    
    /**
     * Swaps the depot with a city. Creates a new route starting
     * in the depot (where the city was) and updates the capacity
     * of the route that is now a city (where the depot was)
     */
    int swap_depot_city(vector<int> *updated_routes, map<int, int> *updated_routes_data, int idx_depot, int city_idx) {
        cout << "merging and breaking" << endl;
        vector<int> temp_routes(*updated_routes);
        map<int, int> temp_route_data(*updated_routes_data);
        swap_cities(&temp_routes, city_idx, idx_depot);
        (temp_route_data).erase(idx_depot);
        
        int route1_start_idx = route_bounds(temp_route_data, city_idx).first;
        int capacity_route1 = route_capacity(temp_routes, temp_route_data, idx_depot);
        int capacity_route2 = route_capacity(temp_routes, temp_route_data, city_idx);
        
        // if swap doesn't violate capacity constraints update routes
        if (capacity_route1 < instance.uniform_vehicle_capacity) {
            swap_cities(updated_routes, city_idx, idx_depot);
            (*updated_routes_data).erase(idx_depot);
            
            (*updated_routes_data)[route1_start_idx] = capacity_route1;
            (*updated_routes_data)[city_idx] = capacity_route2;
            return 1;
        }
        
        return 0;
    }
    
    void swap_cities(vector<int> *updated_routes, int idx1, int idx2) {
        int temp = (*updated_routes)[idx1];
        (*updated_routes)[idx1] = (*updated_routes)[idx2];
        (*updated_routes)[idx2] = temp;
    }
    
    void one_to_one_exchange(vector<int> *updated_routes, map<int, int> *updated_routes_data) {
        int did_exchange = 0;
        while (!did_exchange) {
            // randomly select two indexes to swap
            int idx1 = rand() % updated_routes->size();
            int idx2 = rand() % updated_routes->size();
            
            // find where the route containing the indexes start
            int route1_start_idx = route_bounds(*updated_routes_data, idx1).first;
            int route2_start_idx = route_bounds(*updated_routes_data, idx2).first;
            
            // find the cities in those indexes
            int city_idx1 = (*updated_routes)[idx1];
            int city_idx2 = (*updated_routes)[idx2];
            
            if (idx1 != idx2) { // randomly selected nodes are different
                // vehicle routes are merged
                if (city_idx1 == instance.depot_index) {
                    did_exchange = swap_depot_city(updated_routes, updated_routes_data, idx1, idx2);
                }
                else if (city_idx2 == instance.depot_index) {
                    did_exchange = swap_depot_city(updated_routes, updated_routes_data, idx2, idx1);
                }
                // exchange within the same route - no need to check for capacity constraints
                else if (route1_start_idx == route2_start_idx) {
                    cout << "exchange within same route" << endl;
                    swap_cities(updated_routes, idx1, idx2);
                    did_exchange = 1;
                } else {
                    int capacity_route1 = route_capacity(*updated_routes, *updated_routes_data, idx1);
                    int capacity_route2 = route_capacity(*updated_routes, *updated_routes_data, idx2);
                    
                    int updated_capacity_route1 = capacity_route1 - instance.demands[city_idx1] + instance.demands[city_idx2];
                    int updated_capacity_route2 = capacity_route2 - instance.demands[city_idx2] + instance.demands[city_idx1];
                    
                    if (updated_capacity_route1 < instance.uniform_vehicle_capacity &&
                        updated_capacity_route2 < instance.uniform_vehicle_capacity) {
                        cout << "exchange within different routes" << endl;
                        (*updated_routes_data)[route1_start_idx] = updated_capacity_route1;
                        (*updated_routes_data)[route2_start_idx] = updated_capacity_route2;
                        swap_cities(updated_routes, idx1, idx2);
                        did_exchange = 1;
                    }
                }
            }
        }
    }
    
    int should_update(float cost_diff, float temp) {
        float prob = exp(-cost_diff / temp) * 100;
        return (rand() % 100 < prob);
    }
    
    vector<int> annealing_CVRP() {
        float beta = 1.05; // iteration multiplier
        float initial_temperature = 5000;
        float temperature = initial_temperature;
        const float temp_factor = 0.99;  // temperature reduction multiplier
        const float final_divisor = 50; // terminate iterations if temperature = initial_temp / final_divisor
        const float cutoff_time = 5; // iterations for a given temperature until the next update
        
        initial_solution_greedy();
        cur_route_cost = solution_cost(cur_routes);
        
        best_routes = cur_routes;
        best_route_cost = solution_cost(best_routes);
        
        while (temperature > 0.0001 || temperature) {
            vector<int> updated_routes(cur_routes);
            map<int, int> updated_route_data(cur_routes_data);
            
            update_solution(&updated_routes, &updated_route_data);
            float new_cost = solution_cost(updated_routes);
            float cost_diff = new_cost - cur_route_cost;
            if (cost_diff < 0) { // update improved solution
                cur_routes.assign(updated_routes.begin(), updated_routes.end());
                
                cout << "Cost updated: " << cur_route_cost << "-> " << new_cost << endl;
                cur_route_cost = new_cost;
                if (new_cost < best_route_cost) {
                    cout << "new best!" << endl;
                    best_routes.assign(updated_routes.begin(), updated_routes.end());
                    best_route_cost = new_cost;
                }
            }
            else if (should_update(cost_diff, temperature)) {
                cout << "Cost updated randomly: " << cur_route_cost << "-> " << new_cost << endl;
                
                cur_routes.assign(updated_routes.begin(), updated_routes.end());
                cur_route_cost = new_cost;
            }
            temperature *= temp_factor;
        }
        print_solution(best_routes);
        return best_routes;
    }
};

int main()
{
    cvrp_instance x("instances/X-n101-k25.vrp");
    simulated_annealing annealing(x);
    
    // Testing if load_instance is working as expected
    // x.print_graph();
}
