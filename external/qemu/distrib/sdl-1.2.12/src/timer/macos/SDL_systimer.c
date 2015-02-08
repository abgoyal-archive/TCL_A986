
#include "SDL_config.h"

#ifdef SDL_TIMER_MACOS

#include <Types.h>
#include <Timer.h>
#include <OSUtils.h>
#include <Gestalt.h>
#include <Processes.h>

#include <LowMem.h>

#include "SDL_timer.h"
#include "../SDL_timer_c.h"

#include "FastTimes.h"

#if TARGET_API_MAC_CARBON
#define NewTimerProc NewTimerUPP
#endif

#define MS_PER_TICK	(1000.0/60.0)		/* MacOS tick = 1/60 second */


#define kTwoPower32     (4294967296.0)          /* 2^32 */

static double start_tick;
static int    is_fast_inited = 0;

void SDL_StartTicks(void)
{
        if ( ! is_fast_inited )     // important to check or FastTime may hang machine!
            SDL_SYS_TimerInit();

        start_tick = FastMicroseconds();
}

Uint32 SDL_GetTicks(void)
{
        
        if ( ! is_fast_inited )
            SDL_SYS_TimerInit();
         
        return FastMilliseconds();
}

void SDL_Delay(Uint32 ms)
{
        Uint32 stop, now;

        stop = SDL_GetTicks() + ms;
        do {
            #if TARGET_API_MAC_CARBON
                MPYield();
            #else
                SystemTask();
            #endif

                now = SDL_GetTicks();

        } while ( stop > now );
}

 
/* Data to handle a single periodic alarm */
typedef struct _ExtendedTimerRec
{
	TMTask		     tmTask;
	ProcessSerialNumber  taskPSN;
} ExtendedTimerRec, *ExtendedTimerPtr;

static ExtendedTimerRec gExtendedTimerRec;


int SDL_SYS_TimerInit(void)
{
	FastInitialize ();
	is_fast_inited = 1;
	
	return(0);
}

void SDL_SYS_TimerQuit(void)
{
	/* We don't need a cleanup? */
	return;
}

/* Our Stub routine to set up and then call the real routine. */
pascal void TimerCallbackProc(TMTaskPtr tmTaskPtr)
{
	Uint32 ms;

	WakeUpProcess(&((ExtendedTimerPtr) tmTaskPtr)->taskPSN);

	ms = SDL_alarm_callback(SDL_alarm_interval);
	if ( ms ) {
		SDL_alarm_interval = ROUND_RESOLUTION(ms);
		PrimeTime((QElemPtr)&gExtendedTimerRec.tmTask,
		          SDL_alarm_interval);
	} else {
		SDL_alarm_interval = 0;
	}
}

int SDL_SYS_StartTimer(void)
{
	/*
	 * Configure the global structure that stores the timing information.
	 */
	gExtendedTimerRec.tmTask.qLink = NULL;
	gExtendedTimerRec.tmTask.qType = 0;
	gExtendedTimerRec.tmTask.tmAddr = NewTimerProc(TimerCallbackProc);
	gExtendedTimerRec.tmTask.tmCount = 0;
	gExtendedTimerRec.tmTask.tmWakeUp = 0;
	gExtendedTimerRec.tmTask.tmReserved = 0;
	GetCurrentProcess(&gExtendedTimerRec.taskPSN);

	/* Install the task record */
	InsXTime((QElemPtr)&gExtendedTimerRec.tmTask);

	/* Go! */
	PrimeTime((QElemPtr)&gExtendedTimerRec.tmTask, SDL_alarm_interval);
	return(0);
}

void SDL_SYS_StopTimer(void)
{
	RmvTime((QElemPtr)&gExtendedTimerRec.tmTask);
}

#endif /* SDL_TIMER_MACOS */
