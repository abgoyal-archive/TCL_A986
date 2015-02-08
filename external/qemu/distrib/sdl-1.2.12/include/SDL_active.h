

/* Include file for SDL application focus event handling */

#ifndef _SDL_active_h
#define _SDL_active_h

#include "SDL_stdinc.h"
#include "SDL_error.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

/* The available application states */
#define SDL_APPMOUSEFOCUS	0x01		/* The app has mouse coverage */
#define SDL_APPINPUTFOCUS	0x02		/* The app has input focus */
#define SDL_APPACTIVE		0x04		/* The application is active */

/* Function prototypes */
extern DECLSPEC Uint8 SDLCALL SDL_GetAppState(void);


/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_active_h */
