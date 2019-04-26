#include "time_lib.h"
#include <ctime>
#include <iostream>

using namespace std;

clock_t get_time()
{
    return clock();
}

long double time_in_ms(clock_t start, clock_t end)
{
    clock_t diff = end - start;
    long double total_time = (1000.0 * diff) / CLOCKS_PER_SEC;
    cout << "Total time = " << total_time << " ms" << endl;
    return total_time;
}
