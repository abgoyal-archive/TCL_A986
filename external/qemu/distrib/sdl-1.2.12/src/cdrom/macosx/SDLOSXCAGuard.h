
#include "SDL_config.h"

#if !defined(__CAGuard_h__)
#define __CAGuard_h__


#include <CoreAudio/CoreAudioTypes.h>
#include <pthread.h>



typedef struct S_SDLOSXCAGuard
{

/*  Construction/Destruction */
/*public:*/
/*  Actions */
/*public:*/
    int     (*Lock)(struct S_SDLOSXCAGuard *cag);
    void    (*Unlock)(struct S_SDLOSXCAGuard *cag);
    int     (*Try)(struct S_SDLOSXCAGuard *cag, int *outWasLocked);    /* returns true if lock is free, false if not */
    void    (*Wait)(struct S_SDLOSXCAGuard *cag);
    void    (*Notify)(struct S_SDLOSXCAGuard *cag);

/*  Implementation */
/*protected:*/
    pthread_mutex_t mMutex;
    pthread_cond_t  mCondVar;
    pthread_t       mOwner;
} SDLOSXCAGuard;

SDLOSXCAGuard *new_SDLOSXCAGuard(void);
void delete_SDLOSXCAGuard(SDLOSXCAGuard *cag);

#endif

