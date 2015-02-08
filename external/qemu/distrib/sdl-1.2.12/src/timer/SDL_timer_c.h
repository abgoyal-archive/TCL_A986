
#include "SDL_config.h"

/* Useful functions and variables from SDL_timer.c */
#include "SDL_timer.h"

#define ROUND_RESOLUTION(X)	\
	(((X+TIMER_RESOLUTION-1)/TIMER_RESOLUTION)*TIMER_RESOLUTION)

extern int SDL_timer_started;
extern int SDL_timer_running;

/* Data to handle a single periodic alarm */
extern Uint32 SDL_alarm_interval;
extern SDL_TimerCallback SDL_alarm_callback;

extern int SDL_SetTimerThreaded(int value);

extern int SDL_TimerInit(void);
extern void SDL_TimerQuit(void);

/* This function is called from the SDL event thread if it is available */
extern void SDL_ThreadedTimerCheck(void);
