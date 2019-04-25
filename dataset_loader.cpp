#include "dataset_loader.h"

vector<string> split_line( const string& line )
{
    stringsstream sl(line);
    istream_iterator< string > begin(sl);
    istream_iterator< string > end;
    vector< string > words(begin, end);
    return words;
}

int dataset_loader::euclidean_distance( const pair<int, int>& a, const pair<int, int>& b)
{
    int dx = (a.first - b.first) * (a.first - b.first);
    int dy = (a.second - b.second) * (a.second - b.second);
    return (int) ceil( sqrt( dx + dy ) );
}


