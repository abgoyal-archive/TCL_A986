

/* Include file for SDL quit event handling */

#ifndef _SDL_quit_h
#define _SDL_quit_h

#include "SDL_stdinc.h"
#include "SDL_error.h"


/* There are no functions directly affecting the quit event */
#define SDL_QuitRequested() \
        (SDL_PumpEvents(), SDL_PeepEvents(NULL,0,SDL_PEEKEVENT,SDL_QUITMASK))

#endif /* _SDL_quit_h */
