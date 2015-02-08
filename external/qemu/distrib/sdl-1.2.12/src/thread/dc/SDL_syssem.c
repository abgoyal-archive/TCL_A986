

#include <errno.h>

#include "SDL_config.h"

/* An implementation of semaphores using mutexes and condition variables */

#include "SDL_timer.h"
#include "SDL_thread.h"
#include "SDL_systhread_c.h"


#if SDL_THREADS_DISABLED

SDL_sem *SDL_CreateSemaphore(Uint32 initial_value)
{
	SDL_SetError("SDL not configured with thread support");
	return (SDL_sem *)0;
}

void SDL_DestroySemaphore(SDL_sem *sem)
{
	return;
}

int SDL_SemTryWait(SDL_sem *sem)
{
	SDL_SetError("SDL not configured with thread support");
	return -1;
}

int SDL_SemWaitTimeout(SDL_sem *sem, Uint32 timeout)
{
	SDL_SetError("SDL not configured with thread support");
	return -1;
}

int SDL_SemWait(SDL_sem *sem)
{
	SDL_SetError("SDL not configured with thread support");
	return -1;
}

Uint32 SDL_SemValue(SDL_sem *sem)
{
	return 0;
}

int SDL_SemPost(SDL_sem *sem)
{
	SDL_SetError("SDL not configured with thread support");
	return -1;
}

#else

#include <kos/sem.h>

struct SDL_semaphore
{
	semaphore_t sem;
};

SDL_sem *SDL_CreateSemaphore(Uint32 initial_value)
{
	return (SDL_sem *)sem_create(initial_value);
}

void SDL_DestroySemaphore(SDL_sem *sem)
{
	if ( ! sem ) {
		SDL_SetError("Passed a NULL semaphore");
		return;
	}

	sem_destroy(&sem->sem);
}

int SDL_SemTryWait(SDL_sem *sem)
{
	int retval;

	if ( ! sem ) {
		SDL_SetError("Passed a NULL semaphore");
		return -1;
	}

	retval = sem_trywait(&sem->sem);
	if (retval==0) return 0;
	else return SDL_MUTEX_TIMEDOUT;

	return retval;
}

int SDL_SemWaitTimeout(SDL_sem *sem, Uint32 timeout)
{
	int retval;

	if ( ! sem ) {
		SDL_SetError("Passed a NULL semaphore");
		return -1;
	}

	/* A timeout of 0 is an easy case */
	if ( timeout == 0 ) {
		return SDL_SemTryWait(sem);
	}

	retval = sem_wait_timed(&sem->sem,timeout);
	if (retval==-1) retval= SDL_MUTEX_TIMEDOUT;

	return retval;
}

int SDL_SemWait(SDL_sem *sem)
{
	int retval;

	if ( ! sem ) {
		SDL_SetError("Passed a NULL semaphore");
		return -1;
	}

	while ( ((retval = sem_wait(&sem->sem)) == -1) && (errno == EINTR) ) {}
	return retval;
}

Uint32 SDL_SemValue(SDL_sem *sem)
{
	if ( ! sem ) {
		SDL_SetError("Passed a NULL semaphore");
		return -1;
	}

	return sem_count(&sem->sem);
}

int SDL_SemPost(SDL_sem *sem)
{
	if ( ! sem ) {
		SDL_SetError("Passed a NULL semaphore");
		return -1;
	}

	sem_signal(&sem->sem);
	return 0;
}

#endif /* SDL_THREADS_DISABLED */
