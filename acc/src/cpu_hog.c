/* cpu_hog.c - load generator for the stress test.
 * usage: cpu_hog [priority]   (default 9, below the QNX ACC threads) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>

int main(int argc, char **argv)
{
    int prio = (argc > 1) ? atoi(argv[1]) : 9;
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = prio;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &sp) != 0)
        fprintf(stderr, "cpu_hog: could not set priority %d\n", prio);
    volatile unsigned long x = 0;
    for (;;) x++;
    return 0;
}
