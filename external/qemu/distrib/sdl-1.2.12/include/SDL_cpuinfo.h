

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* CPU feature detection for SDL                                       */

#ifndef _SDL_cpuinfo_h
#define _SDL_cpuinfo_h

#include "SDL_stdinc.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

extern DECLSPEC SDL_bool SDLCALL SDL_HasRDTSC(void);

extern DECLSPEC SDL_bool SDLCALL SDL_HasMMX(void);

extern DECLSPEC SDL_bool SDLCALL SDL_HasMMXExt(void);

extern DECLSPEC SDL_bool SDLCALL SDL_Has3DNow(void);

extern DECLSPEC SDL_bool SDLCALL SDL_Has3DNowExt(void);

extern DECLSPEC SDL_bool SDLCALL SDL_HasSSE(void);

extern DECLSPEC SDL_bool SDLCALL SDL_HasSSE2(void);

extern DECLSPEC SDL_bool SDLCALL SDL_HasAltiVec(void);

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_cpuinfo_h */
