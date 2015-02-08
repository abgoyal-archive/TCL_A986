

/* Simple error message routines for SDL */

#ifndef _SDL_error_h
#define _SDL_error_h

#include "SDL_stdinc.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

/* Public functions */
extern DECLSPEC void SDLCALL SDL_SetError(const char *fmt, ...);
extern DECLSPEC char * SDLCALL SDL_GetError(void);
extern DECLSPEC void SDLCALL SDL_ClearError(void);

/* Private error message function - used internally */
#define SDL_OutOfMemory()	SDL_Error(SDL_ENOMEM)
#define SDL_Unsupported()	SDL_Error(SDL_UNSUPPORTED)
typedef enum {
	SDL_ENOMEM,
	SDL_EFREAD,
	SDL_EFWRITE,
	SDL_EFSEEK,
	SDL_UNSUPPORTED,
	SDL_LASTERROR
} SDL_errorcode;
extern DECLSPEC void SDLCALL SDL_Error(SDL_errorcode code);


/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_error_h */
