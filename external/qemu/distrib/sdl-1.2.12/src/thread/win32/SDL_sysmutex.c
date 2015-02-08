
#include "SDL_config.h"

/* Mutex functions using the Win32 API */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "SDL_mutex.h"


struct SDL_mutex {
	HANDLE id;
};

/* Create a mutex */
SDL_mutex *SDL_CreateMutex(void)
{
	SDL_mutex *mutex;

	/* Allocate mutex memory */
	mutex = (SDL_mutex *)SDL_malloc(sizeof(*mutex));
	if ( mutex ) {
		/* Create the mutex, with initial value signaled */
		mutex->id = CreateMutex(NULL, FALSE, NULL);
		if ( ! mutex->id ) {
			SDL_SetError("Couldn't create mutex");
			SDL_free(mutex);
			mutex = NULL;
		}
	} else {
		SDL_OutOfMemory();
	}
	return(mutex);
}

/* Free the mutex */
void SDL_DestroyMutex(SDL_mutex *mutex)
{
	if ( mutex ) {
		if ( mutex->id ) {
			CloseHandle(mutex->id);
			mutex->id = 0;
		}
		SDL_free(mutex);
	}
}

/* Lock the mutex */
int SDL_mutexP(SDL_mutex *mutex)
{
	if ( mutex == NULL ) {
		SDL_SetError("Passed a NULL mutex");
		return -1;
	}
	if ( WaitForSingleObject(mutex->id, INFINITE) == WAIT_FAILED ) {
		SDL_SetError("Couldn't wait on mutex");
		return -1;
	}
	return(0);
}

/* Unlock the mutex */
int SDL_mutexV(SDL_mutex *mutex)
{
	if ( mutex == NULL ) {
		SDL_SetError("Passed a NULL mutex");
		return -1;
	}
	if ( ReleaseMutex(mutex->id) == FALSE ) {
		SDL_SetError("Couldn't release mutex");
		return -1;
	}
	return(0);
}
