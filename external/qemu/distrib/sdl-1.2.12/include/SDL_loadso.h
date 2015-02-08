

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* System dependent library loading routines                           */



#ifndef _SDL_loadso_h
#define _SDL_loadso_h

#include "SDL_stdinc.h"
#include "SDL_error.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

extern DECLSPEC void * SDLCALL SDL_LoadObject(const char *sofile);

extern DECLSPEC void * SDLCALL SDL_LoadFunction(void *handle, const char *name);

/* Unload a shared object from memory */
extern DECLSPEC void SDLCALL SDL_UnloadObject(void *handle);

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_loadso_h */
