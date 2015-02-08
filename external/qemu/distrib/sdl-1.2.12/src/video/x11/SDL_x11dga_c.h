
#include "SDL_config.h"

#include "SDL_x11video.h"

/* Different DGA access states */
#define DGA_GRAPHICS	0x01
#define DGA_KEYBOARD	0x02
#define DGA_MOUSE	0x04

extern void X11_EnableDGAMouse(_THIS);
extern void X11_CheckDGAMouse(_THIS);
extern void X11_DisableDGAMouse(_THIS);
