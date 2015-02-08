
#include "SDL_config.h"

#ifndef _SDL_ggivideo_h
#define _SDL_ggivideo_h

#include <ggi/ggi.h>

#include "SDL_mouse.h"
#include "../SDL_sysvideo.h"

#define _THIS SDL_VideoDevice *this

/* Private display data */

struct SDL_PrivateVideoData
{
	ggi_visual_t *ggivis;
};

extern ggi_visual_t VIS; /* FIXME: use the private data struct */

extern int SDL_OpenKeyboard(void);
extern void SDL_CloseKeyboard(void);
extern int SDL_OpenMouse(void);
extern void SDL_CloseMouse(void);

#endif /* _SDL_ggivideo_h */
