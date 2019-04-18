#include "data_loader.h"

int main()
{
    string instance_path = "instances/X-n101-k25.vrp";
    instance x = instance( instance_path );
    cout << x.dimension << endl;
    return 0;
}
