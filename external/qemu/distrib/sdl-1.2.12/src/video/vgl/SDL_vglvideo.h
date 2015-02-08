
#include "SDL_config.h"

#ifndef _SDL_vglvideo_h
#define _SDL_vglvideo_h

#include <sys/fbio.h>
#include <sys/consio.h>
#include <vgl.h>

#include "SDL_mouse.h"
#include "SDL_mutex.h"
#include "../SDL_sysvideo.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this

typedef struct {
	int ModeId;
	int Depth;
	int Rmask;
	int Gmask;
	int Bmask;
	VGLBitmap ModeInfo;
} VGLMode;

/* Private display data */
struct SDL_PrivateVideoData {
#define NUM_MODELISTS	4		/* 8, 16, 24, and 32 bits-per-pixel */
	int SDL_nummodes[NUM_MODELISTS];
	SDL_Rect **SDL_modelist[NUM_MODELISTS];
	SDL_mutex *hw_lock;
	VGLMode *VGLCurMode;
	int flip_page;
	byte *flip_address[2];
};
/* Old variable names */
#define SDL_nummodes	(this->hidden->SDL_nummodes)
#define SDL_modelist	(this->hidden->SDL_modelist)
#define hw_lock		(this->hidden->hw_lock)
#define VGLCurMode	(this->hidden->VGLCurMode)
#define flip_page	(this->hidden->flip_page)
#define flip_address	(this->hidden->flip_address)

#endif /* _SDL_vglvideo_h */
