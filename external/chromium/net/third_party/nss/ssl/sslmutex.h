/* $Id: sslmutex.h,v 1.12 2009/06/05 02:34:15 nelson%bolyard.com Exp $ */
#ifndef __SSLMUTEX_H_
#define __SSLMUTEX_H_ 1


#include "prtypes.h"
#include "prlock.h"

#if defined(NETBSD)
#include <sys/param.h> /* for __NetBSD_Version__ */
#endif

#if defined(WIN32)

#include <wtypes.h>

typedef struct 
{
    PRBool isMultiProcess;
#ifdef WINNT
    /* on WINNT we need both the PRLock and the Win32 mutex for fibers */
    struct {
#else
    union {
#endif
        PRLock* sslLock;
        HANDLE sslMutx;
    } u;
} sslMutex;

typedef int    sslPID;

#elif defined(LINUX) || defined(AIX) || defined(BEOS) || defined(BSDI) || (defined(NETBSD) && __NetBSD_Version__ < 500000000) || defined(OPENBSD)

#include <sys/types.h>
#include "prtypes.h"

typedef struct { 
    PRBool isMultiProcess;
    union {
        PRLock* sslLock;
        struct {
            int      mPipes[3]; 
            PRInt32  nWaiters;
        } pipeStr;
    } u;
} sslMutex;
typedef pid_t sslPID;

#elif defined(XP_UNIX) /* other types of Unix */

#include <sys/types.h>	/* for pid_t */
#include <semaphore.h>  /* for sem_t, and sem_* functions */

typedef struct
{
    PRBool isMultiProcess;
    union {
        PRLock* sslLock;
        sem_t  sem;
    } u;
} sslMutex;

typedef pid_t sslPID;

#else

/* what platform is this ?? */

typedef struct { 
    PRBool isMultiProcess;
    union {
        PRLock* sslLock;
        /* include cross-process locking mechanism here */
    } u;
} sslMutex;

typedef int sslPID;

#endif

#include "seccomon.h"

SEC_BEGIN_PROTOS

extern SECStatus sslMutex_Init(sslMutex *sem, int shared);

extern SECStatus sslMutex_Destroy(sslMutex *sem);

extern SECStatus sslMutex_Unlock(sslMutex *sem);

extern SECStatus sslMutex_Lock(sslMutex *sem);

#ifdef WINNT

extern SECStatus sslMutex_2LevelInit(sslMutex *sem);

#endif

SEC_END_PROTOS

#endif
