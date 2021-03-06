
#include "SDL_config.h"

/* An implementation of condition variables using semaphores and mutexes */

#include "SDL_thread.h"

struct SDL_cond
{
	SDL_mutex *lock;
	int waiting;
	int signals;
	SDL_sem *wait_sem;
	SDL_sem *wait_done;
};

/* Create a condition variable */
DECLSPEC SDL_cond * SDLCALL SDL_CreateCond(void)
{
	SDL_cond *cond;

	cond = (SDL_cond *) SDL_malloc(sizeof(SDL_cond));
	if ( cond ) {
		cond->lock = SDL_CreateMutex();
		cond->wait_sem = SDL_CreateSemaphore(0);
		cond->wait_done = SDL_CreateSemaphore(0);
		cond->waiting = cond->signals = 0;
		if ( ! cond->lock || ! cond->wait_sem || ! cond->wait_done ) {
			SDL_DestroyCond(cond);
			cond = NULL;
		}
	} else {
		SDL_OutOfMemory();
	}
	return(cond);
}

/* Destroy a condition variable */
DECLSPEC void SDLCALL SDL_DestroyCond(SDL_cond *cond)
{
	if ( cond ) {
		if ( cond->wait_sem ) {
			SDL_DestroySemaphore(cond->wait_sem);
		}
		if ( cond->wait_done ) {
			SDL_DestroySemaphore(cond->wait_done);
		}
		if ( cond->lock ) {
			SDL_DestroyMutex(cond->lock);
		}
		SDL_free(cond);
	}
}

/* Restart one of the threads that are waiting on the condition variable */
DECLSPEC int SDLCALL SDL_CondSignal(SDL_cond *cond)
{
	if ( ! cond ) {
		SDL_SetError("Passed a NULL condition variable");
		return -1;
	}

	/* If there are waiting threads not already signalled, then
	   signal the condition and wait for the thread to respond.
	*/
	SDL_LockMutex(cond->lock);
	if ( cond->waiting > cond->signals ) {
		++cond->signals;
		SDL_SemPost(cond->wait_sem);
		SDL_UnlockMutex(cond->lock);
		SDL_SemWait(cond->wait_done);
	} else {
		SDL_UnlockMutex(cond->lock);
	}

	return 0;
}

/* Restart all threads that are waiting on the condition variable */
DECLSPEC int SDLCALL SDL_CondBroadcast(SDL_cond *cond)
{
	if ( ! cond ) {
		SDL_SetError("Passed a NULL condition variable");
		return -1;
	}

	/* If there are waiting threads not already signalled, then
	   signal the condition and wait for the thread to respond.
	*/
	SDL_LockMutex(cond->lock);
	if ( cond->waiting > cond->signals ) {
		int i, num_waiting;

		num_waiting = (cond->waiting - cond->signals);
		cond->signals = cond->waiting;
		for ( i=0; i<num_waiting; ++i ) {
			SDL_SemPost(cond->wait_sem);
		}
		/* Now all released threads are blocked here, waiting for us.
		   Collect them all (and win fabulous prizes!) :-)
		 */
		SDL_UnlockMutex(cond->lock);
		for ( i=0; i<num_waiting; ++i ) {
			SDL_SemWait(cond->wait_done);
		}
	} else {
		SDL_UnlockMutex(cond->lock);
	}

	return 0;
}

DECLSPEC int SDLCALL SDL_CondWaitTimeout(SDL_cond *cond, SDL_mutex *mutex, Uint32 ms)
{
	int retval;

	if ( ! cond ) {
		SDL_SetError("Passed a NULL condition variable");
		return -1;
	}

	/* Obtain the protection mutex, and increment the number of waiters.
	   This allows the signal mechanism to only perform a signal if there
	   are waiting threads.
	 */
	SDL_LockMutex(cond->lock);
	++cond->waiting;
	SDL_UnlockMutex(cond->lock);

	/* Unlock the mutex, as is required by condition variable semantics */
	SDL_UnlockMutex(mutex);

	/* Wait for a signal */
	if ( ms == SDL_MUTEX_MAXWAIT ) {
		retval = SDL_SemWait(cond->wait_sem);
	} else {
		retval = SDL_SemWaitTimeout(cond->wait_sem, ms);
	}

	/* Let the signaler know we have completed the wait, otherwise
           the signaler can race ahead and get the condition semaphore
           if we are stopped between the mutex unlock and semaphore wait,
           giving a deadlock.  See the following URL for details:
        http://www-classic.be.com/aboutbe/benewsletter/volume_III/Issue40.html
	*/
	SDL_LockMutex(cond->lock);
	if ( cond->signals > 0 ) {
		/* If we timed out, we need to eat a condition signal */
		if ( retval > 0 ) {
			SDL_SemWait(cond->wait_sem);
		}
		/* We always notify the signal thread that we are done */
		SDL_SemPost(cond->wait_done);

		/* Signal handshake complete */
		--cond->signals;
	}
	--cond->waiting;
	SDL_UnlockMutex(cond->lock);

	/* Lock the mutex, as is required by condition variable semantics */
	SDL_LockMutex(mutex);

	return retval;
}

/* Wait on the condition variable forever */
DECLSPEC int SDLCALL SDL_CondWait(SDL_cond *cond, SDL_mutex *mutex)
{
	return SDL_CondWaitTimeout(cond, mutex, SDL_MUTEX_MAXWAIT);
}
