

#ifndef _SDL_main_h
#define _SDL_main_h

#include "SDL_stdinc.h"

/* Redefine main() on Win32 and MacOS so that it is called by winmain.c */

#if defined(__WIN32__) || \
    (defined(__MWERKS__) && !defined(__BEOS__)) || \
    defined(__MACOS__) || defined(__MACOSX__) || \
    defined(__SYMBIAN32__) || defined(QWS)

#ifdef __cplusplus
#define C_LINKAGE	"C"
#else
#define C_LINKAGE
#endif /* __cplusplus */

#define main	SDL_main

/* The prototype for the application's main() function */
extern C_LINKAGE int SDL_main(int argc, char *argv[]);


/* From the SDL library code -- needed for registering the app on Win32 */
#ifdef __WIN32__

#include "begin_code.h"
#ifdef __cplusplus
extern "C" {
#endif

/* This should be called from your WinMain() function, if any */
extern DECLSPEC void SDLCALL SDL_SetModuleHandle(void *hInst);
/* This can also be called, but is no longer necessary */
extern DECLSPEC int SDLCALL SDL_RegisterApp(char *name, Uint32 style, void *hInst);
/* This can also be called, but is no longer necessary (SDL_Quit calls it) */
extern DECLSPEC void SDLCALL SDL_UnregisterApp(void);
#ifdef __cplusplus
}
#endif
#include "close_code.h"
#endif

/* From the SDL library code -- needed for registering QuickDraw on MacOS */
#if defined(__MACOS__)

#include "begin_code.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration so we don't need to include QuickDraw.h */
struct QDGlobals;

/* This should be called from your main() function, if any */
extern DECLSPEC void SDLCALL SDL_InitQuickDraw(struct QDGlobals *the_qd);

#ifdef __cplusplus
}
#endif
#include "close_code.h"
#endif

#endif /* Need to redefine main()? */

#endif /* _SDL_main_h */
