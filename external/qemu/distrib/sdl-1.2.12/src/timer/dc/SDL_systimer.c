
#include "SDL_config.h"

#ifdef SDL_TIMER_DC

#include <kos.h>

#include "SDL_thread.h"
#include "SDL_timer.h"
#include "../SDL_timer_c.h"

static unsigned start;


void SDL_StartTicks(void)
{
	/* Set first ticks value */
	start = jiffies;
}

Uint32 SDL_GetTicks(void)
{
	return((jiffies-start)*1000/HZ);
}

void SDL_Delay(Uint32 ms)
{
	thd_sleep(ms);
}

/* Data to handle a single periodic alarm */
static int timer_alive = 0;
static SDL_Thread *timer = NULL;

static int RunTimer(void *unused)
{
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
	SDL_SetError("Internal logic error: DC uses threaded timer");
	return(-1);
}

void SDL_SYS_StopTimer(void)
{
	return;
}

#endif /* SDL_TIMER_DC */
