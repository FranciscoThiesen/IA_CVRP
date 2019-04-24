#include "data_loader.h"

constexpr int INF = 0x3f3f3f3f;

vector< string > split_line( string& line )
{
    stringstream sl(line);
    istream_iterator<string> begin(sl);
    istream_iterator<string> end;
    vector<string> words(begin, end);
    return words;
}

inline int euclidean_distance( const pair<int, int>& a, const pair<int, int>& b)
{
    int dx = (a.first - b.first) * (a.first - b.first);
    int dy = (a.second - b.second) * (a.second - b.second);
    return (int) ceil( sqrt( dx + dy) );
}

void instance::initialize_adjacency_matrix()
{
    adjacency_matrix.assign(dimension, vector<int>(dimension, INF) );
    for(int i = 0; i < dimension; ++i)
    {
        for(int j = i + 1; j < dimension; ++j)
        {
            adjacency_matrix[i][j] = adjacency_matrix[j][i] = euclidean_distance(points[i], points[j]);
        }
    }
}

instance::instance( string _path_to_instance )
{
    path_to_instance = _path_to_instance;
    ifstream in(path_to_instance);
    string line;
    vector< vector<string> > file_lines;

    while( getline(in, line) )
    {
        auto x = split_line(line);
        file_lines.emplace_back(x);
    }
    in.close();
    

    instance_name = file_lines[0][2];
    dimension = stoi(file_lines[3][2]);
    uniform_vehicle_capacity = stoi( file_lines[5][2] );

    constexpr int graphdata_start = 7;
        
    for(int node = 0; node < dimension; ++node)
    {
        int x = stoi( file_lines[graphdata_start + node][1] );
        int y = stoi( file_lines[graphdata_start + node][2] );
        points.emplace_back(x, y);
    }

    const int demand_start = graphdata_start + dimension + 1;
    for(int node = 0; node < dimension; ++node)
    {
        int demand = stoi( file_lines[demand_start + node][1] );
        demands.emplace_back( demand );
    }
    
    depot_index = stoi(file_lines[demand_start + dimension + 1][0]) - 1;
    
    initialize_adjacency_matrix();
}

instance::instance() {}

