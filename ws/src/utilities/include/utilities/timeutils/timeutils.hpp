#ifndef TIMEUTILS_HPP
#define TIMEUTILS_HPP

#include <time.h>

void timespec_sum(const timespec& t1, const timespec& t2, timespec& tsum);
void create_tspec(timespec& tdest, double t);
double time_diff(timespec t1, timespec t2);

#endif
