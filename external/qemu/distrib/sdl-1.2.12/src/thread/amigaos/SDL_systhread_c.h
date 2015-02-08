
#include "SDL_config.h"

#include <exec/exec.h>
#include <dos/dos.h>
#include <dos/dostags.h>
#if defined (__SASC) || defined(STORMC4_WOS)
#include <proto/dos.h>
#include <proto/exec.h>
#else
#include <inline/dos.h>
#include <inline/exec.h>
#endif

#include "mydebug.h"

extern struct ExecBase *SysBase;
extern struct DosLibrary *DOSBase;

#ifdef STORMC4_WOS
#include <proto/powerpc.h>

/* use powerpc.library functions instead og exec */
#define SYS_ThreadHandle struct TaskPPC *
#define Signal SignalPPC
#define Wait WaitPPC
#define Task TaskPPC
#define FindTask FindTaskPPC
#define SetSignal SetSignalPPC

#define InitSemaphore InitSemaphorePPC
#define ObtainSemaphore ObtainSemaphorePPC
#define AttemptSemaphore AttemptSemaphorePPC
#define ReleaseSemaphore ReleaseSemaphorePPC
#define SignalSemaphore SignalSemaphorePPC

#else

#define SYS_ThreadHandle struct Task *
#endif /*STORMC4_WOS*/

