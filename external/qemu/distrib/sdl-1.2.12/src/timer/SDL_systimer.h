
#include "SDL_config.h"

/* The system dependent timer handling functions */

#include "SDL_timer.h"
#include "SDL_timer_c.h"


/* Initialize the system dependent timer subsystem */
extern int SDL_SYS_TimerInit(void);

/* Quit the system dependent timer subsystem */
extern void SDL_SYS_TimerQuit(void);

/* Start a timer set up by SDL_SetTimer() */
extern int SDL_SYS_StartTimer(void);

/* Stop a previously started timer */
extern void SDL_SYS_StopTimer(void);
