
#include "SDL_config.h"

#ifndef _SDL_riscosvideo_h
#define _SDL_riscosvideo_h

#include "SDL_mouse.h"
#include "SDL_mutex.h"
#include "../SDL_sysvideo.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this


/* Private display data */

struct SDL_PrivateVideoData {
    unsigned char *bank[2];
    int current_bank;
	unsigned char *alloc_bank;
    int height;
    int xeig;
    int yeig;
	int screen_bpp;
	int screen_width;
	int screen_height;
	char *pixtrans;

	/* Wimp variables */
	unsigned int window_handle;
	char title[256];

#define NUM_MODELISTS	4		/* 8, 16, 24, and 32 bits-per-pixel */
    int SDL_nummodes[NUM_MODELISTS];
    SDL_Rect **SDL_modelist[NUM_MODELISTS];
};

/* Old variable names */
#define SDL_nummodes		(this->hidden->SDL_nummodes)
#define SDL_modelist		(this->hidden->SDL_modelist)

#endif /* _SDL_risosvideo_h */
