


#ifndef VPX_TIMER_H
#define VPX_TIMER_H

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <sys/time.h>

/* timersub is not provided by msys at this time. */
#ifndef timersub
#define timersub(a, b, result) \
    do { \
        (result)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
        (result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
        if ((result)->tv_usec < 0) { \
            --(result)->tv_sec; \
            (result)->tv_usec += 1000000; \
        } \
    } while (0)
#endif
#endif


struct vpx_usec_timer
{
#if defined(_WIN32)
    LARGE_INTEGER  begin, end;
#else
    struct timeval begin, end;
#endif
};


static void
vpx_usec_timer_start(struct vpx_usec_timer *t)
{
#if defined(_WIN32)
    QueryPerformanceCounter(&t->begin);
#else
    gettimeofday(&t->begin, NULL);
#endif
}


static void
vpx_usec_timer_mark(struct vpx_usec_timer *t)
{
#if defined(_WIN32)
    QueryPerformanceCounter(&t->end);
#else
    gettimeofday(&t->end, NULL);
#endif
}


static long
vpx_usec_timer_elapsed(struct vpx_usec_timer *t)
{
#if defined(_WIN32)
    LARGE_INTEGER freq, diff;

    diff.QuadPart = t->end.QuadPart - t->begin.QuadPart;

    if (QueryPerformanceFrequency(&freq) && diff.QuadPart < freq.QuadPart)
        return (long)(diff.QuadPart * 1000000 / freq.QuadPart);

    return 1000000;
#else
    struct timeval diff;

    timersub(&t->end, &t->begin, &diff);
    return diff.tv_sec ? 1000000 : diff.tv_usec;
#endif
}


#endif
