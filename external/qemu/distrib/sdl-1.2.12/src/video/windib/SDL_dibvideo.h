
#include "SDL_config.h"

#ifndef _SDL_dibvideo_h
#define _SDL_dibvideo_h

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

/* for PDA */
typedef enum
{
	SDL_ORIENTATION_UP,
	SDL_ORIENTATION_DOWN,
	SDL_ORIENTATION_LEFT,
	SDL_ORIENTATION_RIGHT
} SDL_ScreenOrientation;

/* Private display data */
struct SDL_PrivateVideoData {
    HBITMAP screen_bmp;
    HPALETTE screen_pal;
    LOGPALETTE *screen_logpal;
    BOOL grab_palette;

    int allow_screensaver;

#define NUM_MODELISTS	4		/* 8, 16, 24, and 32 bits-per-pixel */
    int SDL_nummodes[NUM_MODELISTS];
    SDL_Rect **SDL_modelist[NUM_MODELISTS];

	SDL_ScreenOrientation orientation;
#ifdef _WIN32_WCE
	int invert; /* do to remove, used by GAPI driver! */
	char hiresFix; /* using hires mode without defining hires resource */
	int supportRotation; /* for Pocket PC devices */
	DWORD origRotation; /* for Pocket PC devices */
#endif
};
/* Old variable names */
#define screen_bmp		(this->hidden->screen_bmp)
#define screen_pal		(this->hidden->screen_pal)
#define screen_logpal		(this->hidden->screen_logpal)
#define grab_palette		(this->hidden->grab_palette)
#define SDL_nummodes		(this->hidden->SDL_nummodes)
#define SDL_modelist		(this->hidden->SDL_modelist)

#endif /* _SDL_dibvideo_h */
