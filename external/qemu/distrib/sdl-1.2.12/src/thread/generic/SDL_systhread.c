
#include "SDL_config.h"

/* Thread management routines for SDL */

#include "SDL_thread.h"
#include "../SDL_systhread.h"

int SDL_SYS_CreateThread(SDL_Thread *thread, void *args)
{
	SDL_SetError("Threads are not supported on this platform");
	return(-1);
}

void SDL_SYS_SetupThread(void)
{
	return;
}

Uint32 SDL_ThreadID(void)
{
	return(0);
}

void SDL_SYS_WaitThread(SDL_Thread *thread)
{
	return;
}

void SDL_SYS_KillThread(SDL_Thread *thread)
{
	return;
}

