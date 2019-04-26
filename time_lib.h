#ifndef TIME_LIB_H
#define TIME_LIB_H

#include <iostream>
#include <ctime>

using namespace std;

clock_t get_time();
long double time_in_ms(clock_t start, clock_t end);

#endif
