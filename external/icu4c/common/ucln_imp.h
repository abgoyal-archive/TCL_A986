

#ifndef __UCLN_IMP_H__
#define __UCLN_IMP_H__

#include "ucln.h"
#include <stdlib.h>


#if !UCLN_NO_AUTO_CLEANUP

/*static void ucln_registerAutomaticCleanup();*/
/*static void ucln_unRegisterAutomaticCleanup();*/

/* ------------ automatic cleanup: registration. Choose ONE ------- */
#if defined(UCLN_AUTO_LOCAL)
#include "ucln_local_hook.c"

#elif defined(UCLN_AUTO_ATEXIT)
static UBool gAutoCleanRegistered = FALSE;

static void ucln_atexit_handler()
{
    ucln_cleanupOne(UCLN_TYPE);
}

static void ucln_registerAutomaticCleanup()
{
    if(!gAutoCleanRegistered) {
        gAutoCleanRegistered = TRUE;
        atexit(&ucln_atexit_handler);
    }
}

static void ucln_unRegisterAutomaticCleanup () {
}
/* ------------end of automatic cleanup: registration. ------- */

#elif defined (UCLN_FINI)
U_CAPI void U_EXPORT2 UCLN_FINI (void);

U_CAPI void U_EXPORT2 UCLN_FINI ()
{
    /* This function must be defined, if UCLN_FINI is defined, else link error. */
     ucln_cleanupOne(UCLN_TYPE);
}
#elif defined(__GNUC__)
/* GCC - use __attribute((destructor)) */
static void ucln_destructor()   __attribute__((destructor)) ;

static void ucln_destructor() 
{
    ucln_cleanupOne(UCLN_TYPE);
}

/* Windows: DllMain */
#elif defined (U_WINDOWS)

/* these are from putil.c */
#   define WIN32_LEAN_AND_MEAN
#   define VC_EXTRALEAN
#   define NOUSER
#   define NOSERVICE
#   define NOIME
#   define NOMCX
#   include <windows.h>
BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
    BOOL status = TRUE;

    switch(fdwReason) {
        case DLL_PROCESS_ATTACH:
             /* ICU does not trap process attach, but must pass these through properly. */
            /* ICU specific process attach could go here */
            break;

        case DLL_PROCESS_DETACH:
            /* Here is the one we actually care about. */

            ucln_cleanupOne(UCLN_TYPE);

            break;

        case DLL_THREAD_ATTACH:
            /* ICU does not trap thread attach, but must pass these through properly. */
            /* ICU specific thread attach could go here */
            break;

        case DLL_THREAD_DETACH:
            /* ICU does not trap thread detach, but must pass these through properly. */
            /* ICU specific thread detach could go here */
            break;

    }
    return status;
}
#endif

#endif /* UCLN_NO_AUTO_CLEANUP */

#else
#error This file can only be included once.
#endif
