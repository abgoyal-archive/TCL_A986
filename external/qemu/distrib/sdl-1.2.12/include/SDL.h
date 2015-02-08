

/* Main include header for the SDL library */

#ifndef _SDL_H
#define _SDL_H

#include "SDL_main.h"
#include "SDL_stdinc.h"
#include "SDL_audio.h"
#include "SDL_cdrom.h"
#include "SDL_cpuinfo.h"
#include "SDL_endian.h"
#include "SDL_error.h"
#include "SDL_events.h"
#include "SDL_loadso.h"
#include "SDL_mutex.h"
#include "SDL_rwops.h"
#include "SDL_thread.h"
#include "SDL_timer.h"
#include "SDL_video.h"
#include "SDL_version.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

/* As of version 0.5, SDL is loaded dynamically into the application */

#define	SDL_INIT_TIMER		0x00000001
#define SDL_INIT_AUDIO		0x00000010
#define SDL_INIT_VIDEO		0x00000020
#define SDL_INIT_CDROM		0x00000100
#define SDL_INIT_JOYSTICK	0x00000200
#define SDL_INIT_NOPARACHUTE	0x00100000	/* Don't catch fatal signals */
#define SDL_INIT_EVENTTHREAD	0x01000000	/* Not supported on all OS's */
#define SDL_INIT_EVERYTHING	0x0000FFFF

extern DECLSPEC int SDLCALL SDL_Init(Uint32 flags);

/* This function initializes specific SDL subsystems */
extern DECLSPEC int SDLCALL SDL_InitSubSystem(Uint32 flags);

/* This function cleans up specific SDL subsystems */
extern DECLSPEC void SDLCALL SDL_QuitSubSystem(Uint32 flags);

extern DECLSPEC Uint32 SDLCALL SDL_WasInit(Uint32 flags);

extern DECLSPEC void SDLCALL SDL_Quit(void);

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_H */
