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
    vector<int> cur_routes; // vector containing which node belongs to which routes (the end of the route is delimited by zero)
    map<int, int> cur_routes_data; // contains starting index and capacity for every route
    int cur_route_cost;
    
    vector<int> best_routes;
    int best_route_cost;
    
    simulated_annealing(cvrp_instance ins) {
        instance = ins;
        test_constants();
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
    
    // Fetches in log(n) time beginning and ending indexes for the route which englobes index idx
    pair<int, int> route_bounds(map<int, int> routes_data, int idx) {
        map<int, int>::iterator next_route_start = routes_data.upper_bound(idx);
        pair<int, int> bounds;
        if (next_route_start->first != 0) {
            map<int, int>::iterator route_start = prev(next_route_start);
            bounds = make_pair(route_start->first, next_route_start->first);
        }
        return bounds;
    }
    
    // Given a lower and upper bound gets the route capacity in linear time
    int route_capacity_for_bounds(vector<int> routes, pair<int, int> bounds) {
        int capacity = 0;
        for (int i = bounds.first; i < bounds.second; i++) {
            capacity += instance.demands[routes[i]];
        }
        return capacity;
    }
    
    // Returns total route capacity in log(n) time
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
            start_index = (int)cur_routes.size();
            cur_routes.emplace_back(instance.depot_index);
            total_routes++;
        }
        cur_routes_data[start_index] = 0;
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
    void update_solution(vector<int> &updated_routes, map<int, int> &updated_route_data) {
        int type = rand() % 2;
        switch(type) {
            case 0:
                exchange(updated_routes, updated_route_data);
            case 1:
                delete_and_insert(updated_routes, updated_route_data);
            case 2:
                reverse_route(updated_routes, updated_route_data);
        }
        
    }
    
    
    // EXCHANGE
    /**
     * Swaps the depot with a city. Creates a new route starting
     * in the depot (where the city was - divides an existing route)
     * and updates the capacity of the route that was extended
     * (depot was substituted by a city)
     */
    void swap_cities(vector<int> &updated_routes, int idx1, int idx2) {
        int temp = updated_routes[idx1];
        updated_routes[idx1] = updated_routes[idx2];
        updated_routes[idx2] = temp;
    }
    
    int swap_depot_city(vector<int> &updated_routes, map<int, int> &updated_routes_data, int idx_depot, int city_idx) {
        int joined_route_start_idx = route_bounds(updated_routes_data, idx_depot-1).first;
        int capacity_joined_route1 = updated_routes_data[joined_route_start_idx];
        int capacity_joined_route2 = updated_routes_data[idx_depot];
        int capacity_joined_route = capacity_joined_route1 + capacity_joined_route2 + instance.demands[updated_routes[city_idx]];
        
        // route1 was extended (depot was substituted by a city) and route 2
        // was split (and therefore capacity constraints do not have to be checked)
        if (capacity_joined_route < instance.uniform_vehicle_capacity) {
            int city_route_start_idx = route_bounds(updated_routes_data, city_idx-1).first;
            
            swap_cities(updated_routes, city_idx, idx_depot);
            // erase route starting at depot that was swapped for a city
            updated_routes_data.erase(idx_depot);
            // update joined route capacity
            updated_routes_data[joined_route_start_idx] = capacity_joined_route;
            
            // the city substituted by a depot divided an existing route into two -
            // the capacity of the previous route should be reduced by the capacity of
            // the new divided route
            int capacity_route2 = route_capacity(updated_routes, updated_routes_data, city_idx);
            updated_routes_data[city_idx] = capacity_route2;
            updated_routes_data[city_route_start_idx] -= capacity_route2;
            return 1;
        }
        
        return 0;
    }
    
    void exchange(vector<int> &updated_routes, map<int, int> &updated_routes_data) {
        int did_exchange = 0;
        while (!did_exchange) {
            // randomly select index two indexes to swap - do not allow index 0 or last pos
            int idx1 = (rand() % (updated_routes.size() - 2)) + 1;
            int idx2 = (rand() % (updated_routes.size() - 2)) + 1;
            
            // find where the route containing the indexes start
            pair<int, int> route1_bounds = route_bounds(updated_routes_data, idx1);
            pair<int, int> route2_bounds = route_bounds(updated_routes_data, idx2);
            
            // find the cities in those indexes
            int city_idx1 = updated_routes[idx1];
            int city_idx2 = updated_routes[idx2];
            
            if (city_idx1 != city_idx2) { // randomly selected nodes are different
                // vehicle routes are merged
                if (city_idx1 == instance.depot_index) {
                    did_exchange = swap_depot_city(updated_routes, updated_routes_data, idx1, idx2);
                }
                else if (city_idx2 == instance.depot_index) {
                    did_exchange = swap_depot_city(updated_routes, updated_routes_data, idx2, idx1);
                }
                // exchange within the same route - no need to check for capacity constraints
                else if (route1_bounds.first == route2_bounds.first) {
                    swap_cities(updated_routes, idx1, idx2);
                    did_exchange = 1;
                }
                // exchange within different routes - need to check if capacity is exceeded
                else {
                    int capacity_route1 = updated_routes_data[route1_bounds.first];
                    int capacity_route2 = updated_routes_data[route2_bounds.first];
                    
                    int updated_capacity_route1 = capacity_route1 - instance.demands[city_idx1] + instance.demands[city_idx2];
                    int updated_capacity_route2 = capacity_route2 - instance.demands[city_idx2] + instance.demands[city_idx1];
                    
                    if (updated_capacity_route1 < instance.uniform_vehicle_capacity &&
                        updated_capacity_route2 < instance.uniform_vehicle_capacity) {
                        updated_routes_data[route1_bounds.first] = updated_capacity_route1;
                        updated_routes_data[route2_bounds.first] = updated_capacity_route2;
                        swap_cities(updated_routes, idx1, idx2);
                        did_exchange = 1;
                    }
                }
            }
        }
    }
    
    
    // DELETE AND INSERT
    /**
     * Moves a depot to a different position. Creates a new route starting
     * in the insert position (divides an existing route) and joins adjacent
     * routes that were divided by depot
     */
    int move_depot(vector<int> &updated_routes, map<int, int> &updated_routes_data, int idx_del, int idx_ins) {
        int insert_route_start_idx = route_bounds(updated_routes_data, idx_ins).first;
        int joined_route_start_idx = route_bounds(updated_routes_data, idx_del-1).first;
        int capacity_joined_route;
        if (insert_route_start_idx != joined_route_start_idx) {
            capacity_joined_route = updated_routes_data[joined_route_start_idx] + updated_routes_data[idx_del];
        } else {
            
        }
        
        // route1 was extended (depot was substituted by a city) and route 2
        // was split (and therefore capacity constraints do not have to be checked)
//        if (capacity_joined_route < instance.uniform_vehicle_capacity) {
//            int city_route_start_idx = route_bounds(updated_routes_data, city_idx-1).first;
//
//            move(updated_routes, updated_routes_data, idx_del, idx_ins);
//            // erase route starting at depot that was swapped for a city
//            updated_routes_data.erase(idx_del);
//            // update joined route capacity
//            updated_routes_data[joined_route_start_idx] = capacity_joined_route;
//
//            // the capacity of the previous route should be reduced by the capacity of
//            // the new divided route
//            int capacity_route2 = route_capacity(updated_routes, updated_routes_data, city_idx);
//            updated_routes_data[city_idx] = capacity_route2;
//            updated_routes_data[city_route_start_idx] -= capacity_route2;
//            return 1;
//        }
//
        return 0;
    }
    
    void move(vector<int> &updated_routes, map<int, int> &updated_routes_data, int idx_del, int idx_ins) {
        int moved_city = updated_routes[idx_del];
        pair<int, int> route_del_bounds = route_bounds(updated_routes_data, idx_del);
        pair<int, int> route_ins_bounds = route_bounds(updated_routes_data, idx_ins);
        int capacity_route_ins = updated_routes_data[route_ins_bounds.first] + instance.demands[moved_city];
        int capacity_route_del = updated_routes_data[route_del_bounds.first] - instance.demands[moved_city];
        
        // capacities are not updated if both deleted and inserted are in same route
        int in_same_route = route_del_bounds.first == route_ins_bounds.first;
        
        if (idx_ins < idx_del) { // shift right
            for (int i = idx_del; i > idx_ins; i--) {
                updated_routes[i] = updated_routes[i-1];
                if (updated_routes[i] == instance.depot_index) {
                    // route capacity stays the same as before, but index is shifted
                    updated_routes_data[i] = updated_routes_data[i-1];
                    updated_routes_data.erase(i-1);
                }
            }
            // if deleted element was in a later route than where it was inserted
            // the first element in its original route is pushed back by one
            if (!in_same_route) {
                updated_routes_data[route_del_bounds.first+1] = capacity_route_del;
                updated_routes_data[route_ins_bounds.first] = capacity_route_ins;
            }
        } else { // shift left
            for (int i = idx_del; i < idx_ins; i++) {
                updated_routes[i] = updated_routes[i+1];
                if (updated_routes[i] == instance.depot_index) {
                    // route capacity stays the same as before, but index is shifted
                    updated_routes_data[i] = updated_routes_data[i+1];
                    updated_routes_data.erase(i+1);
                }
            }
            // if deleted element was originally at a position before the one where
            // it was inserted the first element in the inserted route is one before
            if (!in_same_route) {
                updated_routes_data[route_del_bounds.first] = capacity_route_del;
                updated_routes_data[route_ins_bounds.first-1] = capacity_route_ins;
            }
            
        }
        updated_routes[idx_ins] = moved_city;
    }
    
    void delete_and_insert(vector<int> &updated_routes, map<int, int> &updated_routes_data) {
        int did_exchange = 0;
        while (!did_exchange) {
            // randomly select index to delete and idx to insert - do not allow index 0 or last pos
            int idx_del = (rand() % (updated_routes.size() - 2)) + 1;
            int idx_insert = (rand() % (updated_routes.size() - 2)) + 1;
            
            pair<int, int> route_del_bounds = route_bounds(updated_routes_data, idx_del);
            pair<int, int> route_ins_bounds = route_bounds(updated_routes_data, idx_insert);
            
            // find the cities in those indexes
            int city_del = updated_routes[idx_del];
            int pos_ins = updated_routes[idx_insert];
            
            if (idx_del != idx_insert) {
                // vehicle routes are merged
                if (city_del == instance.depot_index) {
//                    did_exchange = move_depot(updated_routes, updated_routes_data, idx_del, idx_insert);
                } else if (pos_ins == instance.depot_index) {
                    
                }
                // delete and add within the same route - no need to check for capacity constraints
                else if (route_del_bounds.first == route_ins_bounds.first) {
                    move(updated_routes, updated_routes_data, idx_del, idx_insert);
                    did_exchange = 1;
                }
                // add to different routes - need to check if capacity is exceeded
                else {
                    int capacity_route_ins = updated_routes_data[route_ins_bounds.first] + instance.demands[city_del];
                    
                    if (capacity_route_ins < instance.uniform_vehicle_capacity) {
                        move(updated_routes, updated_routes_data, idx_del, idx_insert);
                        did_exchange = 1;
                    }
                }
            }
            
            if (did_exchange) {
                check_routes_data(updated_routes, updated_routes_data);
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
    
    void reverse_route(vector<int> &routes, map<int, int> &route_data) {
        int idx = (rand() % (routes.size() - 2)) + 1;
        pair<int, int> bounds = route_bounds(route_data, idx);
        
        vector<int> route;
        route.assign(routes.begin() + bounds.first, routes.begin() + bounds.second);
        
        int route_size = bounds.second - bounds.first;
        if (route_size) {
            int best_distance = solution_cost(route);
            
            for (int i = 1; i < route_size - 1; i++) {
                for (int k = i + 1; k < route_size; k++) {
                    vector<int> new_route = reverse_aux(route, i, k);
                    int new_distance = solution_cost(new_route);
                    if (new_distance < best_distance) {
                        route = new_route;
                    }
                }
            }
        }
        
        copy(route.begin(), route.end(), routes.begin() + bounds.first);
    }
    
    /*
    * Simulated Annealing
    */
    int should_update(float cost_diff, float temp) {
        float prob = exp(-cost_diff / temp) * 100;
        return (rand() % 100 < prob);
    }
    
    vector<int> annealing_CVRP(float initial_temperature, float temp_factor) {
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
            vector<int> updated_routes(cur_routes);
            map<int, int> updated_route_data(cur_routes_data);
            
            update_solution(updated_routes, updated_route_data);
            float new_cost = solution_cost(updated_routes);
            float cost_diff = new_cost - cur_route_cost;
            if (cost_diff < 0) { // update improved solution
                time_since_improvement = 0;
                cur_routes = updated_routes;
                cur_routes_data = updated_route_data;
                cur_route_cost = new_cost;
                cout << "Cost updated: " << cur_route_cost << "-> " << new_cost << endl;
                if (new_cost < best_route_cost) {
                    cout << "new best!" << endl;
                    best_routes.assign(updated_routes.begin(), updated_routes.end());
                    best_route_cost = new_cost;
                }
            }
            else if (cost_diff != 0 && should_update(cost_diff, temperature)) {
                cout << "Cost updated randomly: " << cur_route_cost << "-> " << new_cost << endl;
                
                cur_routes = updated_routes;
                cur_routes_data = updated_route_data;
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
        vector<float> initial_temperatures = {10000, 9000, 8000, 6000, 5000, 4000, 3000, 2000, 1000, 500};
        vector<float> temp_factors = {0.85, 0.9, 0.95};
        float best_params_cost = 10e5;
        float best_temp, best_factor;
        for (int t = 0; t < initial_temperatures.size(); t++) {
            for (int f = 0; f < temp_factors.size(); f++) {
                annealing_CVRP(initial_temperatures[t], temp_factors[t]);
                if (best_route_cost < best_params_cost) {
                    best_params_cost = best_route_cost;
                    best_temp = initial_temperatures[t];
                    best_factor = temp_factors[f];
                }
            }
        }
        printf("Best temp: %.2f, best factor: %.2f\n", best_temp, best_factor);
    }
    
    void check_routes_data(vector<int> routes, map<int, int> routes_data) {
        int capacity = 0;
        int last_route_start = 0;
        for (int i = 1; i < routes.size(); i++) {
            if (routes[i] == 0) {
                if (routes_data[last_route_start] != capacity) {
                    printf("start: %d, expected: %d, actual: %d\n", last_route_start, capacity, routes_data[last_route_start]);
                }
                last_route_start = i;
                capacity = 0;
            }
            capacity += instance.demands[routes[i]];
        }
    }
};

int main()
{
    cvrp_instance x("instances/X-n101-k25.vrp");
    simulated_annealing annealing(x);
}
