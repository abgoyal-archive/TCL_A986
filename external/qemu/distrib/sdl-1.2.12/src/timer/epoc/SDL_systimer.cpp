
#include "SDL_config.h"

#ifdef SDL_TIMER_EPOC


#include <e32std.h>
#include <e32hal.h>

extern "C" {
#include "SDL_error.h"
#include "SDL_thread.h"
#include "SDL_timer.h"
#include "../SDL_timer_c.h"

static TUint start = 0;
static TInt tickPeriodMilliSeconds;


void SDL_StartTicks(void)
{
	/* Set first ticks value */
    start = User::TickCount();

    TTimeIntervalMicroSeconds32 period;
	TInt tmp = UserHal::TickPeriod(period);
    tickPeriodMilliSeconds = period.Int() / 1000;
}

Uint32 SDL_GetTicks(void)
{
    TUint deltaTics = User::TickCount() - start;
	return(deltaTics * tickPeriodMilliSeconds); 
}

void SDL_Delay(Uint32 ms)
{
     
    User::After(TTimeIntervalMicroSeconds32(ms*1000));
}

/* Data to handle a single periodic alarm */
static int timer_alive = 0;
static SDL_Thread *timer = NULL;

static int RunTimer(void *unused)
{
	Uint32 ms;

	while ( timer_alive ) {
		if ( SDL_timer_running ) {
			SDL_ThreadedTimerCheck();
		}
		SDL_Delay(10);
	}
	return(0);
}

/* This is only called if the event thread is not running */
int SDL_SYS_TimerInit(void)
{
	timer_alive = 1;
	timer = SDL_CreateThread(RunTimer, NULL);
	if ( timer == NULL )
		return(-1);
	return(SDL_SetTimerThreaded(1));
}

void SDL_SYS_TimerQuit(void)
{
	timer_alive = 0;
	if ( timer ) {
		SDL_WaitThread(timer, NULL);
		timer = NULL;
	}
}

int SDL_SYS_StartTimer(void)
{
	SDL_SetError("Internal logic error: Epoc uses threaded timer");
	return(-1);
}

void SDL_SYS_StopTimer(void)
{
	return;
}

}; // extern "C"

#endif /* SDL_TIMER_EPOC */
