
#include "SDL_config.h"



#include "SDL_stdinc.h"

/*#define NDEBUG 1*/
#define assert(X)


#include "SDLOSXCAGuard.h"

/*#warning      Need a try-based Locker too*/

static int SDLOSXCAGuard_Lock(SDLOSXCAGuard *cag)
{
    int theAnswer = 0;
    
    if(pthread_self() != cag->mOwner)
    {
        OSStatus theError = pthread_mutex_lock(&cag->mMutex);
        (void)theError;
        assert(theError == 0);
        cag->mOwner = pthread_self();
        theAnswer = 1;
    }

    return theAnswer;
}

static void    SDLOSXCAGuard_Unlock(SDLOSXCAGuard *cag)
{
    OSStatus theError;
    assert(pthread_self() == cag->mOwner);

    cag->mOwner = 0;
    theError = pthread_mutex_unlock(&cag->mMutex);
    (void)theError;
    assert(theError == 0);
}

static int SDLOSXCAGuard_Try (SDLOSXCAGuard *cag, int *outWasLocked)
{
    int theAnswer = 0;
    *outWasLocked = 0;
    
    if (pthread_self() == cag->mOwner) {
        theAnswer = 1;
        *outWasLocked = 0;
    } else {
        OSStatus theError = pthread_mutex_trylock(&cag->mMutex);
        if (theError == 0) {
            cag->mOwner = pthread_self();
            theAnswer = 1;
            *outWasLocked = 1;
        }
    }
    
    return theAnswer;
}

static void    SDLOSXCAGuard_Wait(SDLOSXCAGuard *cag)
{
    OSStatus theError;
    assert(pthread_self() == cag->mOwner);

    cag->mOwner = 0;

    theError = pthread_cond_wait(&cag->mCondVar, &cag->mMutex);
    (void)theError;
    assert(theError == 0);
    cag->mOwner = pthread_self();
}

static void    SDLOSXCAGuard_Notify(SDLOSXCAGuard *cag)
{
    OSStatus theError = pthread_cond_signal(&cag->mCondVar);
    (void)theError;
    assert(theError == 0);
}


SDLOSXCAGuard *new_SDLOSXCAGuard(void)
{
    OSStatus theError;
    SDLOSXCAGuard *cag = (SDLOSXCAGuard *) SDL_malloc(sizeof (SDLOSXCAGuard));
    if (cag == NULL)
        return NULL;
    SDL_memset(cag, '\0', sizeof (*cag));

    #define SET_SDLOSXCAGUARD_METHOD(m) cag->m = SDLOSXCAGuard_##m
    SET_SDLOSXCAGUARD_METHOD(Lock);
    SET_SDLOSXCAGUARD_METHOD(Unlock);
    SET_SDLOSXCAGUARD_METHOD(Try);
    SET_SDLOSXCAGUARD_METHOD(Wait);
    SET_SDLOSXCAGUARD_METHOD(Notify);
    #undef SET_SDLOSXCAGUARD_METHOD

    theError = pthread_mutex_init(&cag->mMutex, NULL);
    (void)theError;
    assert(theError == 0);
    
    theError = pthread_cond_init(&cag->mCondVar, NULL);
    (void)theError;
    assert(theError == 0);
    
    cag->mOwner = 0;
    return cag;
}

void delete_SDLOSXCAGuard(SDLOSXCAGuard *cag)
{
    if (cag != NULL)
    {
        pthread_mutex_destroy(&cag->mMutex);
        pthread_cond_destroy(&cag->mCondVar);
        SDL_free(cag);
    }
}

