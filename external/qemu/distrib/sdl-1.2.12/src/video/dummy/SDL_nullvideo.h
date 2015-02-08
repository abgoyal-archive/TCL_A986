
#include "SDL_config.h"

#ifndef _SDL_nullvideo_h
#define _SDL_nullvideo_h

#include "../SDL_sysvideo.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this


/* Private display data */

struct SDL_PrivateVideoData {
    int w, h;
    void *buffer;
};

#endif /* _SDL_nullvideo_h */
