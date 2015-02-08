
#include "SDL_config.h"



/* Mutex functions using the Win32 API */

#include<e32std.h>

#include "SDL_error.h"
#include "SDL_mutex.h"



struct SDL_mutex
    {
    TInt handle;
    };

extern TInt CreateUnique(TInt (*aFunc)(const TDesC& aName, TAny*, TAny*), TAny*, TAny*);

TInt NewMutex(const TDesC& aName, TAny* aPtr1, TAny*)
    {
    return ((RMutex*)aPtr1)->CreateGlobal(aName);
    }

/* Create a mutex */
SDL_mutex *SDL_CreateMutex(void)
{
    RMutex rmutex;

    TInt status = CreateUnique(NewMutex, &rmutex, NULL);
	if(status != KErrNone)
	    {
			SDL_SetError("Couldn't create mutex");
		}
    SDL_mutex* mutex = new /*(ELeave)*/ SDL_mutex;
    mutex->handle = rmutex.Handle();
	return(mutex);
}

/* Free the mutex */
void SDL_DestroyMutex(SDL_mutex *mutex)
{
	if ( mutex ) 
	{
    RMutex rmutex;
    rmutex.SetHandle(mutex->handle);
	rmutex.Signal();
	rmutex.Close();
	delete(mutex);
    mutex = NULL;
	}
}

/* Lock the mutex */
int SDL_mutexP(SDL_mutex *mutex)
{
	if ( mutex == NULL ) {
		SDL_SetError("Passed a NULL mutex");
		return -1;
	}
    RMutex rmutex;
    rmutex.SetHandle(mutex->handle);
	rmutex.Wait(); 
	return(0);
}

/* Unlock the mutex */
int SDL_mutexV(SDL_mutex *mutex)
{
	if ( mutex == NULL ) {
		SDL_SetError("Passed a NULL mutex");
		return -1;
	}
	RMutex rmutex;
    rmutex.SetHandle(mutex->handle);
	rmutex.Signal();
	return(0);
}
