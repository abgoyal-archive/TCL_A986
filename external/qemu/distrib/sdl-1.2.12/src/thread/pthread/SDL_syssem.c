
#include "SDL_config.h"

#include <pthread.h>
#include <semaphore.h>
#include <errno.h>

#include "SDL_thread.h"
#include "SDL_timer.h"

/* Wrapper around POSIX 1003.1b semaphores */

#ifdef __MACOSX__
/* Mac OS X doesn't support sem_getvalue() as of version 10.4 */
#include "../generic/SDL_syssem.c"
#else

struct SDL_semaphore {
	sem_t sem;
};

/* Create a semaphore, initialized with value */
SDL_sem *SDL_CreateSemaphore(Uint32 initial_value)
{
	SDL_sem *sem = (SDL_sem *) SDL_malloc(sizeof(SDL_sem));
	if ( sem ) {
		if ( sem_init(&sem->sem, 0, initial_value) < 0 ) {
			SDL_SetError("sem_init() failed");
			SDL_free(sem);
			sem = NULL;
		}
	} else {
		SDL_OutOfMemory();
	}
	return sem;
}

void SDL_DestroySemaphore(SDL_sem *sem)
{
	if ( sem ) {
		sem_destroy(&sem->sem);
		SDL_free(sem);
	}
}

int SDL_SemTryWait(SDL_sem *sem)
{
	int retval;

	if ( ! sem ) {
		SDL_SetError("Passed a NULL semaphore");
		return -1;
	}
	retval = SDL_MUTEX_TIMEDOUT;
	if ( sem_trywait(&sem->sem) == 0 ) {
		retval = 0;
	}
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
	if ( retval < 0 ) {
		SDL_SetError("sem_wait() failed");
	}
	return retval;
}

int SDL_SemWaitTimeout(SDL_sem *sem, Uint32 timeout)
{
	int retval;

	if ( ! sem ) {
		SDL_SetError("Passed a NULL semaphore");
		return -1;
	}

	/* Try the easy cases first */
	if ( timeout == 0 ) {
		return SDL_SemTryWait(sem);
	}
	if ( timeout == SDL_MUTEX_MAXWAIT ) {
		return SDL_SemWait(sem);
	}

	/* Ack!  We have to busy wait... */
	/* FIXME: Use sem_timedwait()? */
	timeout += SDL_GetTicks();
	do {
		retval = SDL_SemTryWait(sem);
		if ( retval == 0 ) {
			break;
		}
		SDL_Delay(1);
	} while ( SDL_GetTicks() < timeout );

	return retval;
}

Uint32 SDL_SemValue(SDL_sem *sem)
{
	int ret = 0;
	if ( sem ) {
		sem_getvalue(&sem->sem, &ret);
		if ( ret < 0 ) {
			ret = 0;
		}
	}
	return (Uint32)ret;
}

int SDL_SemPost(SDL_sem *sem)
{
	int retval;

	if ( ! sem ) {
		SDL_SetError("Passed a NULL semaphore");
		return -1;
	}

	retval = sem_post(&sem->sem);
	if ( retval < 0 ) {
		SDL_SetError("sem_post() failed");
	}
	return retval;
}

#endif /* __MACOSX__ */
