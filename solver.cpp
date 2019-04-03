#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

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
};


int main()
{
    cvrp_instance x("instances/X-n101-k25.vrp");
    

    // Testing if load_instance is working as expected
    x.print_graph();
}
