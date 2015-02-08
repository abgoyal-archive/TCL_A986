

#ifndef _SDL_timer_h
#define _SDL_timer_h

/* Header for the SDL time management routines */

#include "SDL_stdinc.h"
#include "SDL_error.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

/* This is the OS scheduler timeslice, in milliseconds */
#define SDL_TIMESLICE		10

/* This is the maximum resolution of the SDL timer on all platforms */
#define TIMER_RESOLUTION	10	/* Experimentally determined */

extern DECLSPEC Uint32 SDLCALL SDL_GetTicks(void);

/* Wait a specified number of milliseconds before returning */
extern DECLSPEC void SDLCALL SDL_Delay(Uint32 ms);

/* Function prototype for the timer callback function */
typedef Uint32 (SDLCALL *SDL_TimerCallback)(Uint32 interval);

extern DECLSPEC int SDLCALL SDL_SetTimer(Uint32 interval, SDL_TimerCallback callback);


typedef Uint32 (SDLCALL *SDL_NewTimerCallback)(Uint32 interval, void *param);

/* Definition of the timer ID type */
typedef struct _SDL_TimerID *SDL_TimerID;

extern DECLSPEC SDL_TimerID SDLCALL SDL_AddTimer(Uint32 interval, SDL_NewTimerCallback callback, void *param);

extern DECLSPEC SDL_bool SDLCALL SDL_RemoveTimer(SDL_TimerID t);

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_timer_h */
