
#include "SDL_config.h"

/* Matrox hardware acceleration for the SDL framebuffer console driver */

#include "SDL_fbvideo.h"

/* Set up the driver for Matrox acceleration */
extern void FB_MatroxAccel(_THIS, __u32 card);
