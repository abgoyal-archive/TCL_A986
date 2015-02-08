
#include "SDL_config.h"

/* This is the definition of the YUV video surface function structure */

#include "SDL_video.h"
#include "SDL_sysvideo.h"

#ifndef _THIS
#define _THIS	SDL_VideoDevice *_this
#endif
struct private_yuvhwfuncs {
	int (*Lock)(_THIS, SDL_Overlay *overlay);
	void (*Unlock)(_THIS, SDL_Overlay *overlay);
	int (*Display)(_THIS, SDL_Overlay *overlay, SDL_Rect *src, SDL_Rect *dst);
	void (*FreeHW)(_THIS, SDL_Overlay *overlay);
};
