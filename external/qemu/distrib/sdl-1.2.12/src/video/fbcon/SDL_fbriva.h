
#include "SDL_config.h"

/* Riva hardware acceleration for the SDL framebuffer console driver */

#include "SDL_fbvideo.h"

#ifndef FB_ACCEL_NV3
#define FB_ACCEL_NV3	27
#endif
#ifndef FB_ACCEL_NV4
#define FB_ACCEL_NV4	28
#endif

/* Set up the driver for Riva acceleration */
extern void FB_RivaAccel(_THIS, __u32 card);
