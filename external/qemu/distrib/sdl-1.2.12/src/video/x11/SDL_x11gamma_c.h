
#include "SDL_config.h"

#ifndef _SDL_x11gamma_h
#define _SDL_x11gamma_h

extern int X11_SetVidModeGamma(_THIS, float red, float green, float blue);
extern int X11_GetVidModeGamma(_THIS, float *red, float *green, float *blue);
extern void X11_SaveVidModeGamma(_THIS);
extern void X11_SwapVidModeGamma(_THIS);

#endif
