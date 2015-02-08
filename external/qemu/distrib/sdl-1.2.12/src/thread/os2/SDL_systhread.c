
#include "SDL_config.h"

/* OS/2 thread management routines for SDL */

#include <process.h>
#define INCL_DOSERRORS
#define INCL_DOSPROCESS
#include <os2.h>

#include "SDL_thread.h"
#include "../SDL_systhread.h"
#include "../SDL_thread_c.h"

typedef struct ThreadStartParms
{
  void *args;
  pfnSDL_CurrentEndThread pfnCurrentEndThread;
} tThreadStartParms, *pThreadStartParms;

static void threadfunc(void *pparm)
{
  pThreadStartParms pThreadParms = pparm;
  pfnSDL_CurrentEndThread pfnCurrentEndThread = NULL;

  // Call the thread function!
  SDL_RunThread(pThreadParms->args);

  // Get the current endthread we have to use!
  if (pThreadParms)
  {
    pfnCurrentEndThread = pThreadParms->pfnCurrentEndThread;
    SDL_free(pThreadParms);
  }
  // Call endthread!
  if (pfnCurrentEndThread)
    (*pfnCurrentEndThread)();
}

int SDL_SYS_CreateThread(SDL_Thread *thread, void *args, pfnSDL_CurrentBeginThread pfnBeginThread, pfnSDL_CurrentEndThread pfnEndThread)
{
  pThreadStartParms pThreadParms = SDL_malloc(sizeof(tThreadStartParms));
  if (!pThreadParms)
  {
    SDL_SetError("Not enough memory to create thread");
    return(-1);
  }

  // Save the function which we will have to call to clear the RTL of calling app!
  pThreadParms->pfnCurrentEndThread = pfnEndThread;
  // Also save the real parameters we have to pass to thread function
  pThreadParms->args = args;
  // Start the thread using the runtime library of calling app!
  thread->threadid = thread->handle = (*pfnBeginThread)(threadfunc, NULL, 512*1024, pThreadParms);
  if ((int)thread->threadid <= 0)
  {
    SDL_SetError("Not enough resources to create thread");
    return(-1);
  }
  return(0);
}

void SDL_SYS_SetupThread(void)
{
  return;
}

DECLSPEC Uint32 SDLCALL SDL_ThreadID(void)
{
  PTIB tib;
  DosGetInfoBlocks(&tib, NULL);
  return((Uint32) (tib->tib_ptib2->tib2_ultid));
}

void SDL_SYS_WaitThread(SDL_Thread *thread)
{
  TID tid = thread->handle;
  DosWaitThread(&tid, DCWW_WAIT);
}

void SDL_SYS_KillThread(SDL_Thread *thread)
{
  DosKillThread(thread->handle);
}
