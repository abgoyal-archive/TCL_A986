


#include <e32std.h>
#include <e32def.h>
#include <e32svr.h>
#include <e32base.h>
#include <estlib.h>
#include <stdlib.h>
#include <stdio.h>
#include <w32std.h>
#include <apgtask.h>

#include "SDL_error.h"

#ifndef EXPORT_C
# ifdef __VC32__
#  define IMPORT_C __declspec(dllexport)
#  define EXPORT_C __declspec(dllexport)
# endif
# ifdef __GCC32__
#  define IMPORT_C
#  define EXPORT_C __declspec(dllexport)
# endif
#endif

#if defined(__WINS__)
#include <estw32.h>
IMPORT_C void RegisterWsExe(const TDesC &aName);
#endif

/* The prototype for the application's main() function */
#define main	SDL_main
extern "C" int main (int argc, char *argv[], char *envp[]);
extern "C" void exit (int ret);


/* Epoc main function */

GLDEF_C TInt E32Main() 
    {
    /*  Get the clean-up stack */
	CTrapCleanup* cleanup = CTrapCleanup::New();

    #if defined(__WINS__)
    /* arrange for access to Win32 stdin/stdout/stderr */
   	RWin32Stream::StartServer();	
    #endif

    /* Arrange for multi-threaded operation */
	SpawnPosixServerThread();	

    /* Get args and environment */
	int argc=0;
	char** argv=0;
	char** envp=0;
	__crt0(argc,argv,envp);	

    #if defined(__WINS__)
	/* Cause the graphical Window Server to come into existence */
	RSemaphore sem;
	sem.CreateGlobal(_L("WsExeSem"),0);
	RegisterWsExe(sem.FullName());
    #endif


    /* Start the application! */

    /* Create stdlib */
    _REENT;

    /* Set process and thread priority */
    RThread currentThread;

    currentThread.Rename(_L("SdlProgram"));
    currentThread.SetProcessPriority(EPriorityLow);
    currentThread.SetPriority(EPriorityMuchLess);

    /* Call stdlib main */
    int ret = main(argc, argv, envp); /* !! process exits here if there is "exit()" in main! */	

    /* Call exit */
    exit(ret); /* !! process exits here! */			

    /* Free resources and return */
    CloseSTDLIB();
   	delete cleanup;									
    return(KErrNone);
    }

/* Epoc dll entry point */
#if defined(__WINS__)
GLDEF_C TInt E32Dll(TDllReason)
	{
	return(KErrNone);
	}

EXPORT_C TInt WinsMain(TAny *)
	{
    E32Main();
	return KErrNone;
	}
#endif
