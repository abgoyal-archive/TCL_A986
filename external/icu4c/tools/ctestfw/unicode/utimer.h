
#ifndef _UTIMER_H
#define _UTIMER_H

#include "unicode/utypes.h"

#if defined(U_WINDOWS)
#   define VC_EXTRALEAN
#   define WIN32_LEAN_AND_MEAN
#   include <windows.h>
#else
#   if defined(OS390)
#     define __UU  /* Universal Unix - for struct timeval */
#   endif
#   include <time.h>
#   include <sys/time.h> 
#   include <unistd.h> 
#endif


typedef struct UTimer UTimer;

typedef void FuntionToBeTimed(void* param);


#if defined(U_WINDOWS)

    struct UTimer{
        LARGE_INTEGER start;
        LARGE_INTEGER placeHolder;
    };      
        
    int uprv_initFrequency(UTimer* timer)
    {
        return QueryPerformanceFrequency(&timer->placeHolder);
    }
    void uprv_start(UTimer* timer)
    {
        QueryPerformanceCounter(&timer->start);
    }
    double uprv_delta(UTimer* timer1, UTimer* timer2){
        return ((double)(timer2->start.QuadPart - timer1->start.QuadPart))/((double)timer1->placeHolder.QuadPart);
    }
    UBool uprv_compareFrequency(UTimer* timer1, UTimer* timer2){
        return (timer1->placeHolder.QuadPart == timer2->placeHolder.QuadPart);
    }

#else

    struct UTimer{
        struct timeval start;
        struct timeval placeHolder;
    };
    
    int32_t uprv_initFrequency(UTimer* /*timer*/)
    {
        return 0;
    }
    void uprv_start(UTimer* timer)
    {
        gettimeofday(&timer->start, 0);
    }
    double uprv_delta(UTimer* timer1, UTimer* timer2){
        double t1, t2;

        t1 =  (double)timer1->start.tv_sec + (double)timer1->start.tv_usec/(1000*1000);
        t2 =  (double)timer2->start.tv_sec + (double)timer2->start.tv_usec/(1000*1000);
        return (t2-t1);
    }
    UBool uprv_compareFrequency(UTimer* /*timer1*/, UTimer* /*timer2*/){
        return TRUE;
    }

#endif
static U_INLINE void U_EXPORT2
utimer_getTime(UTimer* timer){
    uprv_initFrequency(timer);
    uprv_start(timer);
}

static U_INLINE double U_EXPORT2
utimer_getDeltaSeconds(UTimer* timer1, UTimer* timer2){
    if(uprv_compareFrequency(timer1,timer2)){
        return uprv_delta(timer1,timer2);
    }
    /* got error return -1 */
    return -1;
}

static U_INLINE double U_EXPORT2
utimer_getElapsedSeconds(UTimer* timer){
    UTimer temp;
    utimer_getTime(&temp);
    return uprv_delta(timer,&temp);
}

static U_INLINE double U_EXPORT2
utimer_loopUntilDone(double thresholdTimeVal,
                     int32_t* loopCount, 
                     FuntionToBeTimed fn, 
                     void* param){
    UTimer timer;
    double currentVal=0;
    *loopCount = 0;
    utimer_getTime(&timer);
    for(;currentVal<thresholdTimeVal;){
        fn(param);
        currentVal = utimer_getElapsedSeconds(&timer);
        *loopCount++;
    }
    return currentVal;
}

#endif

