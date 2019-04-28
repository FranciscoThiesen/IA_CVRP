#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <utility>
#include <iterator>
#include <map>
#include <unordered_map>
#include <climits>
#include <random>
#include <numeric>
#include "data_loader.h"
#include "neighborhood_generator.h"
#include "time_lib.h"
#include <fstream>

#define TRACE(x) 

constexpr int INF = 0x3f3f3f3f;

using namespace std;

inline int euclidean_distance( const pair<int, int>& a, const pair<int, int>& b)
{
    int dx = (a.first - b.first) * (a.first - b.first);
    int dy = (a.second - b.second) * (a.second - b.second);
    return (int) ceil( sqrt( dx + dy ) );
}

struct grasp_solver
{
    instance test_data; // Instancia que sera resolvida     
    pair<int, int> center; // Posicao geografica do deposito
    int center_idx; // Indice do deposito
    neighborhood_generator n_generator; // gerador de vizinhanca para uma solucao 
    vector< vector<int> > cur_routes; // routas da solucao atual
    vector<int> cur_routes_capacities; // demandas sendo atentidadas em cada rota
    int cur_routes_cost; // custo da solucao atual 
    
    vector< vector<int> > best_routes; 
    int best_routes_cost;
    
    // Funcao que calcula o custo total de uma solucao, uma rota de cada vez 
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
    
    /*
     * A funcao smart_greedy foi chave na obtencao de solucoes proximas do OPT
     * Nela realizamos os seguintes passos
     * 1 - Ordenamos os pontos por suas coordenadas geograficas, ordenando em relacao ao angulo que cada ponto faz com o deposito
     * 2 - Realizarmos um shift circular nesse vetor ordenado. ( via funcao de c++ rotate )
     * 3 - Seguimos a ordem obtida, tentando sempre inserir o proximo elemento na ultima rota criada ate o momento. 
     *     Se nao for possivel atender a esse elemento por questoes de capacidade dos caminhoes, iniciamos uma nova rota que contem
     *     esse novo elemento.
     */
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
        
        int rot = ( rand() % test_data.dimension);
        
        // Shift circular do vetor ordenado radialmente
        rotate(points.begin(), points.begin() + rot, points.end() );
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
   
    /* Essa versao do solver obedece a politica de first_improvement. 
     * Como essa versao obteve resultados estritamente piores nas instancias, vamos nos limitar
     * a analisar a versao que segue a politiva de best_improvement
     */

    vector< vector<int> > cvrp_solver_first_improvement(const int max_stall_iterations, vector<int>& neighborhood_set, int seed) 
    {
        smart_greedy();
        best_routes = cur_routes;
        best_routes_cost = cur_routes_cost;
        int cur_stall_iterations = 0;
        
        n_generator.set_seed(seed);

        while(cur_stall_iterations < max_stall_iterations)
        {
            vector< vector<int> > updated_routes = cur_routes;
            vector<int> updated_routes_capacities = cur_routes_capacities;
            
            n_generator.update_solution_custom(updated_routes, updated_routes_capacities, neighborhood_set);
            
            int new_cost = solution_cost(updated_routes);
            int improvement = cur_routes_cost - new_cost;
            
            if( improvement > 0 )
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

    /* Essa versao do solver foi que obteve os melhores resultados.
     * Passos:
     * 1 - Gerar solucao inicial via smart_greedy
     * 2 - A cada step, selecionamos de forma equiprovável uma das seguintes vizinhancas ( delete_and_insert e exchange )
     * 3 - Buscamos o melhor vizinho da vizinhanca escolhida no passo 2.
     * 4 - Se esse vizinho é estritamente melhor que a solucao atual, atribuímos ele a nossa solução inicial
     * 5 - Se apos, max_stall_iterations nao obtivemos melhora a melhor solucao. Retornamos a melhor solucao encontrada
     */

    vector< vector<int> > cvrp_solver_best_improvement(const int max_stall_iterations, int seed) 
    {
        smart_greedy();
        best_routes = cur_routes;
        best_routes_cost = cur_routes_cost;
        int cur_stall_iterations = 0;
        
        n_generator.set_seed(seed);

        while(cur_stall_iterations < max_stall_iterations)
        {
            vector< vector<int> > updated_routes = cur_routes;
            vector<int> updated_routes_capacities = cur_routes_capacities;
            bool has_improved = n_generator.update_solution_best_improvement(updated_routes, updated_routes_capacities);
            if( has_improved ) {
                best_routes = updated_routes;
                best_routes_cost = solution_cost( best_routes );
                cur_stall_iterations = 0;
                cur_routes = best_routes;
                cur_routes_capacities = updated_routes_capacities;
                cur_routes_cost = best_routes_cost;
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

// Funcao que chama o solver com parametros definidos
int generate_solution( string instance_name, int allowed_iterations )
{
    grasp_solver solver( instance_name );
    int best_cost = INF;
    vector< vector<int> > best_solution_found;
    
    srand(13); // Lucky seed (:
    
    for(int i = 0; i < allowed_iterations; ++i)
    {
        int s = rand();
        auto solution = solver.cvrp_solver_best_improvement(10, s);
        int solution_cost = solver.solution_cost( solution );
        if( solution_cost < best_cost ) 
        {
            best_cost = solution_cost;
            best_solution_found = solution;
        }
    }
    return best_cost;
}


int main()
{
    
    instance first_instance("instances/X-n204-k19.vrp");
    string instance_prefix = "instances/";
    string csv_prefix = "results/";
    vector< string > instances = { "X-n101-k25.vrp", "X-n110-k13.vrp", "X-n115-k10.vrp", "X-n204-k19.vrp" };
    vector< string > csv_names = { "X-n101-k25.csv", "X-n110-k13.csv", "X-n115-k10.csv", "X-n204-k19.csv" };
    vector< int > bks = { 27591, 14971, 12747, 19565 };
    vector< int > iterations = { 5, 10, 50, 100, 500, 1000, 10000 };
    
    constexpr int total_instances = 4;
    
    for(int i = 0; i < total_instances; ++i) 
    {
        string instance_name = instance_prefix + instances[i];
        string file_name = csv_prefix + csv_names[i];
        ofstream out_file(file_name);
        cout << "Rodando para a imagem " << instances[i] << endl;
        out_file << "Total iteracoes,Tempo total(ms),Solucao encontrada,BKS,Approximation Ratio" << endl;
        for(const int iter : iterations )
        {
            cout << "rodando para uma quantidade de iteracoes = " << iter << endl;
            clock_t start = get_time();
            int solution_cost = generate_solution( instance_name, iter );
            clock_t end   = get_time();
            long double duration = time_in_ms(start, end);
            out_file << iter << "," << duration << "," << solution_cost << "," << bks[i] << "," << (1.0 * solution_cost / bks[i] ) << endl; 
        }
        out_file.close();
    }
    

    return 0;
}
