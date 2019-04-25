#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <utility>
#include <iterator>
#include <map>
#include <unordered_map>
#include <climits>
#include "data_loader.h"
#include "neighborhood_generator.h"

#define TRACE(x) 

using namespace std;

inline int euclidean_distance( const pair<int, int>& a, const pair<int, int>& b)
{
    int dx = (a.first - b.first) * (a.first - b.first);
    int dy = (a.second - b.second) * (a.second - b.second);
    return (int) ceil( sqrt( dx + dy ) );
}

struct grasp_solver
{
    instance test_data;     
    pair<int, int> center;
    int center_idx;
    neighborhood_generator n_generator;
    vector< vector<int> > cur_routes;
    vector<int> cur_routes_capacities;
    int cur_routes_cost; 
    
    vector< vector<int> > best_routes;
    int best_routes_cost;
    

    int solution_cost(const vector< vector<int> >& routes)
    {
        int total_cost = 0;
        
        auto cost_per_route = [&] ( const vector<int>& route )
        {
            int cost = 0;
            
            for(int i = 1; i < (int) route.size(); ++i)
                cost += euclidean_distance( test_data.points[route[i]], test_data.points[route[i - 1] ] );
            
            cost += euclidean_distance( test_data.points[center_idx], test_data.points[route.back()] );
            return cost;
        };

        for(const auto& R : routes) total_cost += cost_per_route( R ); 
       
        return total_cost;
    }


    void worst_solver_ever()
    {
        cur_routes.clear();
        cur_routes_capacities.clear();

        for(int i = 0; i < test_data.dimension; ++i)
        {
            if( i != center_idx )
            {
                cur_routes.emplace_back( vector<int>(1, center_idx) );
                cur_routes.back().push_back(i);
                cur_routes_capacities.emplace_back( test_data.demands[i] ); 
            }
        }
        cur_routes_cost = solution_cost(cur_routes);
    }

    void smart_greedy()
    {
        cur_routes.clear();
        cur_routes_capacities.clear();

        vector< vector<int> > routes(1, vector<int>(1, center_idx));
        vector< int > route_demand(1, 0);
        vector< tuple<int, int, int> > points;
        for(int p = 0; p < test_data.dimension; ++p)
        {
            if( p == test_data.depot_index ) continue;
            points.emplace_back( test_data.points[p].first, test_data.points[p].second, p );
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
            if( test_data.demands[point_index] + route_demand[current_route] <= test_data.uniform_vehicle_capacity )
            {
                route_demand[current_route] += test_data.demands[point_index];
                routes[current_route].push_back( point_index );
            }
            else
            {
                current_route++;
                route_demand.push_back(0);
                route_demand[current_route] += test_data.demands[point_index];
                routes.push_back( vector<int>(1, center_idx) );
                routes.back().push_back( point_index ); 
            
            }
        }
        
        cur_routes = routes; 
        cur_routes_capacities = route_demand; 
        cur_routes_cost = solution_cost(cur_routes);
    }
    
    /* Testaremos varios tipos de solvers
     * 1 - Best improvement
     * 2 - Any improvement
     * 3 - Greedy burro / Greedy "inteligente" randomizado
     * 4 - Todas as combinacoes de vizinhanca possiveis
    */
    
    // Pretendemos reportar todas as variacoes descritas acima
    
    vector< vector<int> > grasp_solver_dumb_start(const int max_stall_iterations) 
    {
        //worst_solver_ever();
        smart_greedy();
        best_routes = cur_routes;
        best_routes_cost = cur_routes_cost;
        cout << (int) best_routes.size() << endl;
        int cur_stall_iterations = 0;
        while(cur_stall_iterations < max_stall_iterations)
        {
            vector< vector<int> > updated_routes = cur_routes;
            vector<int> updated_routes_capacities = cur_routes_capacities;
            n_generator.update_solution(updated_routes, updated_routes_capacities);
            int new_cost = solution_cost(updated_routes);
            int improvement = new_cost - cur_routes_cost;
            if( improvement < 0 )
            {
                cur_routes = updated_routes;
                cur_routes_capacities = updated_routes_capacities;
                cur_routes_cost = new_cost;
                cur_stall_iterations = 0;
                best_routes = cur_routes;
                best_routes_cost = cur_routes_cost;
            }
            else cur_stall_iterations++;
        }
        return best_routes;
    }

    

    grasp_solver(instance _test_data )
    {
        test_data = _test_data;
        center = test_data.points[test_data.depot_index];
        center_idx = test_data.depot_index;
        n_generator = neighborhood_generator(test_data);
    }
    
    grasp_solver() {}

};

int main()
{
    instance first_instance("instances/X-n101-k25.vrp");
    grasp_solver solver(first_instance);
    int instance_dimension = first_instance.dimension;
    vector< vector< int > > best_so_far;
    int cost_best_solve = 0x3f3f3f3f;

    auto best_solution = solver.grasp_solver_dumb_start(100);
    
    cout << cost_best_solve << endl;

    return 0;
}
