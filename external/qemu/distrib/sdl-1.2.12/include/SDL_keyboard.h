

/* Include file for SDL keyboard event handling */

#ifndef _SDL_keyboard_h
#define _SDL_keyboard_h

#include "SDL_stdinc.h"
#include "SDL_error.h"
#include "SDL_keysym.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

typedef struct SDL_keysym {
	Uint8 scancode;			/* hardware specific scancode */
	SDLKey sym;			/* SDL virtual keysym */
	SDLMod mod;			/* current key modifiers */
	Uint16 unicode;			/* translated character */
} SDL_keysym;

/* This is the mask which refers to all hotkey bindings */
#define SDL_ALL_HOTKEYS		0xFFFFFFFF

/* Function prototypes */
extern DECLSPEC int SDLCALL SDL_EnableUNICODE(int enable);

#define SDL_DEFAULT_REPEAT_DELAY	500
#define SDL_DEFAULT_REPEAT_INTERVAL	30
extern DECLSPEC int SDLCALL SDL_EnableKeyRepeat(int delay, int interval);
extern DECLSPEC void SDLCALL SDL_GetKeyRepeat(int *delay, int *interval);

extern DECLSPEC Uint8 * SDLCALL SDL_GetKeyState(int *numkeys);

extern DECLSPEC SDLMod SDLCALL SDL_GetModState(void);

extern DECLSPEC void SDLCALL SDL_SetModState(SDLMod modstate);

extern DECLSPEC char * SDLCALL SDL_GetKeyName(SDLKey key);


/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_keyboard_h */
