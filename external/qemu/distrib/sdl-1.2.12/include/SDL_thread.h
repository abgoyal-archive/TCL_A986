

#ifndef _SDL_thread_h
#define _SDL_thread_h


#include "SDL_stdinc.h"
#include "SDL_error.h"

/* Thread synchronization primitives */
#include "SDL_mutex.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

/* The SDL thread structure, defined in SDL_thread.c */
struct SDL_Thread;
typedef struct SDL_Thread SDL_Thread;

/* Create a thread */
#if ((defined(__WIN32__) && !defined(HAVE_LIBC)) || defined(__OS2__)) &&  !defined(__SYMBIAN32__)
#define SDL_PASSED_BEGINTHREAD_ENDTHREAD
#ifndef _WIN32_WCE
#include <process.h> /* This has _beginthread() and _endthread() defined! */
#endif

#ifdef __OS2__
typedef int (*pfnSDL_CurrentBeginThread)(void (*func)(void *), void *, unsigned, void *arg); 
typedef void (*pfnSDL_CurrentEndThread)(void);
#elif __GNUC__
typedef unsigned long (__cdecl *pfnSDL_CurrentBeginThread) (void *, unsigned,
        unsigned (__stdcall *func)(void *), void *arg, 
        unsigned, unsigned *threadID);
typedef void (__cdecl *pfnSDL_CurrentEndThread)(unsigned code);
#else
typedef uintptr_t (__cdecl *pfnSDL_CurrentBeginThread) (void *, unsigned,
        unsigned (__stdcall *func)(void *), void *arg, 
        unsigned, unsigned *threadID);
typedef void (__cdecl *pfnSDL_CurrentEndThread)(unsigned code);
#endif

extern DECLSPEC SDL_Thread * SDLCALL SDL_CreateThread(int (SDLCALL *fn)(void *), void *data, pfnSDL_CurrentBeginThread pfnBeginThread, pfnSDL_CurrentEndThread pfnEndThread);

#ifdef __OS2__
#define SDL_CreateThread(fn, data) SDL_CreateThread(fn, data, _beginthread, _endthread)
#elif defined(_WIN32_WCE)
#define SDL_CreateThread(fn, data) SDL_CreateThread(fn, data, NULL, NULL)
#else
#define SDL_CreateThread(fn, data) SDL_CreateThread(fn, data, _beginthreadex, _endthreadex)
#endif
#else
extern DECLSPEC SDL_Thread * SDLCALL SDL_CreateThread(int (SDLCALL *fn)(void *), void *data);
#endif

/* Get the 32-bit thread identifier for the current thread */
extern DECLSPEC Uint32 SDLCALL SDL_ThreadID(void);

extern DECLSPEC Uint32 SDLCALL SDL_GetThreadID(SDL_Thread *thread);

extern DECLSPEC void SDLCALL SDL_WaitThread(SDL_Thread *thread, int *status);

/* Forcefully kill a thread without worrying about its state */
extern DECLSPEC void SDLCALL SDL_KillThread(SDL_Thread *thread);


/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_thread_h */
