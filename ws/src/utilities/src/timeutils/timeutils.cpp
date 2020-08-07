#include <math.h>
#include "utilities/timeutils/timeutils.hpp"
void timespec_sum(const timespec& t1, const timespec& t2, timespec& tsum) {
    long int nsec = t1.tv_nsec + t2.tv_nsec;
    long int sec = t1.tv_sec + t2.tv_sec; 

    if (nsec > 1e9) {
        nsec -= 1e9;
        sec++;
    }

    tsum.tv_nsec = nsec;
    tsum.tv_sec = sec;
}

void create_tspec(timespec& tdest, double t) {
    double intpart = floor(t);
    tdest.tv_sec = (long int)intpart;
    tdest.tv_nsec = (long int)((t - intpart) * 1e9);
}

double time_diff(timespec t1, timespec t2) {
    long int nsec = (t1.tv_nsec - t2.tv_nsec);
    long int sec = (t1.tv_sec - t2.tv_sec);
    if (nsec < 0) {
        sec -= 1;
        nsec = 1e9 + nsec;
    } 		

    return (double)(sec + (double)nsec/1e9); 
}
