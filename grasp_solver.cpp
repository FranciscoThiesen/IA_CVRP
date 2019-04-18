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

    pair<int, vector< vector<int> > > smart_greedy()
    {
        vector< vector<int> > routes(1, vector<int>());
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
        // Lets try to create routes similar to "petals". We should check for demand feasibility for each route
        for(const auto& P : points) 
        {
            int point_index = get<2>(P);
            if( test_data.demands[point_index] + route_demand[current_route] <= test_data.uniform_vehicle_capacity )
            {
                route_demand[current_route] += test_data.demands[point_index];
                routes[current_route].emplace_back( point_index );
            }
            else
            {
                current_route++;
                route_demand[current_route] += test_data.demands[point_index];
                routes.emplace_back( vector<int>(1, point_index) );
            }
        }
       
        int total_route_cost = 0;
        for(const auto& R : routes )
        {
            int route_length = (int) R.size();
            for(int idx = 1; idx < route_length; ++idx) total_route_cost += euclidean_distance( test_data.points[R[idx]], test_data.points[R[idx - 1]] );
            total_route_cost += euclidean_distance( test_data.points[R[test_data.depot_index]], test_data.points[R[0]]);
            total_route_cost += euclidean_distance( test_data.points[R[test_data.depot_index]], test_data.points[R.back()]);
        
        }

        return make_pair(total_route_cost, routes);

    }
    

    grasp_solver(instance _test_data )
    {
        test_data = _test_data;
        center = test_data.points[test_data.depot_index];
    }
    
    grasp_solver() {}

};

int main()
{

}
