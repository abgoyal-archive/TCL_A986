
#include "SDL_config.h"

/* Mutex functions using the OS/2 API */

#define INCL_DOSERRORS
#define INCL_DOSSEMAPHORES
#include <os2.h>

#include "SDL_mutex.h"


struct SDL_mutex {
	HMTX hmtxID;
};

/* Create a mutex */
DECLSPEC SDL_mutex * SDLCALL SDL_CreateMutex(void)
{
  SDL_mutex *mutex;
  APIRET ulrc;

  /* Allocate mutex memory */
  mutex = (SDL_mutex *)SDL_malloc(sizeof(*mutex));
  if (mutex)
  {
    /* Create the mutex, with initial value signaled */
    ulrc = DosCreateMutexSem(NULL,                  // Create unnamed semaphore
                             &(mutex->hmtxID),      // Pointer to handle
                             0L,                    // Flags: create it private (not shared)
                             FALSE);                // Initial value: unowned
    if (ulrc!=NO_ERROR)
    {
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
DECLSPEC void SDLCALL SDL_DestroyMutex(SDL_mutex *mutex)
{
  if ( mutex )
  {
    if ( mutex->hmtxID )
    {
      DosCloseMutexSem(mutex->hmtxID);
      mutex->hmtxID = 0;
    }
    SDL_free(mutex);
  }
}

/* Lock the mutex */
DECLSPEC int SDLCALL SDL_mutexP(SDL_mutex *mutex)
{
  if ( mutex == NULL )
  {
    SDL_SetError("Passed a NULL mutex");
    return -1;
  }
  if ( DosRequestMutexSem(mutex->hmtxID, SEM_INDEFINITE_WAIT) != NO_ERROR )
  {
    SDL_SetError("Couldn't wait on mutex");
    return -1;
  }
  return(0);
}

/* Unlock the mutex */
DECLSPEC int SDLCALL SDL_mutexV(SDL_mutex *mutex)
{
  if ( mutex == NULL )
  {
    SDL_SetError("Passed a NULL mutex");
    return -1;
  }
  if ( DosReleaseMutexSem(mutex->hmtxID) != NO_ERROR )
  {
    SDL_SetError("Couldn't release mutex");
    return -1;
  }
  return(0);
}
