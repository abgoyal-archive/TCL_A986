
#include "SDL_config.h"

/* 3Dfx hardware acceleration for the SDL framebuffer console driver */

#include "SDL_fbvideo.h"

/* Set up the driver for 3Dfx acceleration */
extern void FB_3DfxAccel(_THIS, __u32 card);
