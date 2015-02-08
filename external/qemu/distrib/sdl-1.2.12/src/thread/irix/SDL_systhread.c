
#include "SDL_config.h"

/* IRIX thread management routines for SDL */

#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/prctl.h>

#include "SDL_thread.h"
#include "../SDL_systhread.h"


static int sig_list[] = {
	SIGHUP, SIGINT, SIGQUIT, SIGPIPE, SIGALRM, SIGTERM, SIGCLD, SIGWINCH,
	SIGVTALRM, SIGPROF, 0
};


int SDL_SYS_CreateThread(SDL_Thread *thread, void *args)
{
	/* Create the thread and go! */
	if ( sproc(SDL_RunThread, PR_SALL, args) < 0 ) {
		SDL_SetError("Not enough resources to create thread");
		return(-1);
	}
	return(0);
}

void SDL_SYS_SetupThread(void)
{
	int i;
	sigset_t mask;

	/* Mask asynchronous signals for this thread */
	sigemptyset(&mask);
	for ( i=0; sig_list[i]; ++i ) {
		sigaddset(&mask, sig_list[i]);
	}
	sigprocmask(SIG_BLOCK, &mask, NULL);
}

/* WARNING:  This may not work for systems with 64-bit pid_t */
Uint32 SDL_ThreadID(void)
{
	return((Uint32)getpid());
}

/* WARNING:  This may not work for systems with 64-bit pid_t */
void SDL_WaitThread(SDL_Thread *thread, int *status)
{
	errno = 0;
	while ( errno != ECHILD ) {
		waitpid(thread->handle, NULL, 0);
	}
}

/* WARNING:  This may not work for systems with 64-bit pid_t */
void SDL_KillThread(SDL_Thread *thread)
{
	kill(thread->handle, SIGKILL);
}

