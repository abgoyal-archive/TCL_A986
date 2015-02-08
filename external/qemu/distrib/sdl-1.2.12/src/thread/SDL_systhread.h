
#include "SDL_config.h"

/* These are functions that need to be implemented by a port of SDL */

#ifndef _SDL_systhread_h
#define _SDL_systhread_h

#include "SDL_thread.h"

#ifdef SDL_PASSED_BEGINTHREAD_ENDTHREAD
extern int SDL_SYS_CreateThread(SDL_Thread *thread, void *args, pfnSDL_CurrentBeginThread pfnBeginThread, pfnSDL_CurrentEndThread pfnEndThread);
#else
extern int SDL_SYS_CreateThread(SDL_Thread *thread, void *args);
#endif

/* This function does any necessary setup in the child thread */
extern void SDL_SYS_SetupThread(void);

extern void SDL_SYS_WaitThread(SDL_Thread *thread);

/* This function kills the thread and returns */
extern void SDL_SYS_KillThread(SDL_Thread *thread);

#endif /* _SDL_systhread_h */
