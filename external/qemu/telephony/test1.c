
#include "sysdeps.h"
#include <stdio.h>

#define  MAX_COUNTER  10

static int  counter = 0;

static void
timer_func( void*  _timer )
{
    SysTimer  timer = _timer;
    SysTime   now   = sys_time_ms();

    ++counter;
    printf( "tick %d/%d a %.2fs\n", counter, MAX_COUNTER, now/1000. );
    if (counter < MAX_COUNTER)
        sys_timer_set( timer, now + 2000, timer_func, timer );
    else
        sys_timer_destroy( timer );
}


int  main( void )
{
    SysTimer  timer;

    /* initialize event subsystem */
    sys_main_init();

    /* create timer and register it */
    timer = sys_timer_create();
    sys_timer_set( timer, sys_time_ms() + 1000, timer_func, timer );

    printf("entering event loop\n");
    sys_main_loop();
    printf("exiting event loop\n" );
    return 0;
}
