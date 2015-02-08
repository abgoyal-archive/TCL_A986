

/* This header defines the current SDL version */

#ifndef _SDL_version_h
#define _SDL_version_h

#include "SDL_stdinc.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

#define SDL_MAJOR_VERSION	1
#define SDL_MINOR_VERSION	2
#define SDL_PATCHLEVEL		12

typedef struct SDL_version {
	Uint8 major;
	Uint8 minor;
	Uint8 patch;
} SDL_version;

#define SDL_VERSION(X)							\
{									\
	(X)->major = SDL_MAJOR_VERSION;					\
	(X)->minor = SDL_MINOR_VERSION;					\
	(X)->patch = SDL_PATCHLEVEL;					\
}

#define SDL_VERSIONNUM(X, Y, Z)						\
	((X)*1000 + (Y)*100 + (Z))

/* This is the version number macro for the current SDL version */
#define SDL_COMPILEDVERSION \
	SDL_VERSIONNUM(SDL_MAJOR_VERSION, SDL_MINOR_VERSION, SDL_PATCHLEVEL)

/* This macro will evaluate to true if compiled with SDL at least X.Y.Z */
#define SDL_VERSION_ATLEAST(X, Y, Z) \
	(SDL_COMPILEDVERSION >= SDL_VERSIONNUM(X, Y, Z))

extern DECLSPEC const SDL_version * SDLCALL SDL_Linked_Version(void);

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_version_h */
