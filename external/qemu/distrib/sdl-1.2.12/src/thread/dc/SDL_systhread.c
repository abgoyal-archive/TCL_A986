
#include "SDL_config.h"

/* Thread management routines for SDL */

#include "SDL_thread.h"
#include "../SDL_thread_c.h"
#include "../SDL_systhread.h"

#include <kos/thread.h>

int SDL_SYS_CreateThread(SDL_Thread *thread, void *args)
{
	thread->handle = thd_create(SDL_RunThread,args);
	if (thread->handle == NULL) {
		SDL_SetError("Not enough resources to create thread");
		return(-1);
	}
	return(0);
}

void SDL_SYS_SetupThread(void)
{
	return;
}

Uint32 SDL_ThreadID(void)
{
	return (Uint32)thd_get_current();
}

void SDL_SYS_WaitThread(SDL_Thread *thread)
{
	thd_wait(thread->handle);
}

void SDL_SYS_KillThread(SDL_Thread *thread)
{
	thd_destroy(thread->handle);
}
