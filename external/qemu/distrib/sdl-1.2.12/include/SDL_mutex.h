

#ifndef _SDL_mutex_h
#define _SDL_mutex_h


#include "SDL_stdinc.h"
#include "SDL_error.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

#define SDL_MUTEX_TIMEDOUT	1

/* This is the timeout value which corresponds to never time out */
#define SDL_MUTEX_MAXWAIT	(~(Uint32)0)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Mutex functions                                               */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* The SDL mutex structure, defined in SDL_mutex.c */
struct SDL_mutex;
typedef struct SDL_mutex SDL_mutex;

/* Create a mutex, initialized unlocked */
extern DECLSPEC SDL_mutex * SDLCALL SDL_CreateMutex(void);

/* Lock the mutex  (Returns 0, or -1 on error) */
#define SDL_LockMutex(m)	SDL_mutexP(m)
extern DECLSPEC int SDLCALL SDL_mutexP(SDL_mutex *mutex);

#define SDL_UnlockMutex(m)	SDL_mutexV(m)
extern DECLSPEC int SDLCALL SDL_mutexV(SDL_mutex *mutex);

/* Destroy a mutex */
extern DECLSPEC void SDLCALL SDL_DestroyMutex(SDL_mutex *mutex);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Semaphore functions                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* The SDL semaphore structure, defined in SDL_sem.c */
struct SDL_semaphore;
typedef struct SDL_semaphore SDL_sem;

/* Create a semaphore, initialized with value, returns NULL on failure. */
extern DECLSPEC SDL_sem * SDLCALL SDL_CreateSemaphore(Uint32 initial_value);

/* Destroy a semaphore */
extern DECLSPEC void SDLCALL SDL_DestroySemaphore(SDL_sem *sem);

extern DECLSPEC int SDLCALL SDL_SemWait(SDL_sem *sem);

extern DECLSPEC int SDLCALL SDL_SemTryWait(SDL_sem *sem);

extern DECLSPEC int SDLCALL SDL_SemWaitTimeout(SDL_sem *sem, Uint32 ms);

extern DECLSPEC int SDLCALL SDL_SemPost(SDL_sem *sem);

/* Returns the current count of the semaphore */
extern DECLSPEC Uint32 SDLCALL SDL_SemValue(SDL_sem *sem);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Condition variable functions                                  */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* The SDL condition variable structure, defined in SDL_cond.c */
struct SDL_cond;
typedef struct SDL_cond SDL_cond;

/* Create a condition variable */
extern DECLSPEC SDL_cond * SDLCALL SDL_CreateCond(void);

/* Destroy a condition variable */
extern DECLSPEC void SDLCALL SDL_DestroyCond(SDL_cond *cond);

extern DECLSPEC int SDLCALL SDL_CondSignal(SDL_cond *cond);

extern DECLSPEC int SDLCALL SDL_CondBroadcast(SDL_cond *cond);

extern DECLSPEC int SDLCALL SDL_CondWait(SDL_cond *cond, SDL_mutex *mut);

extern DECLSPEC int SDLCALL SDL_CondWaitTimeout(SDL_cond *cond, SDL_mutex *mutex, Uint32 ms);

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_mutex_h */
