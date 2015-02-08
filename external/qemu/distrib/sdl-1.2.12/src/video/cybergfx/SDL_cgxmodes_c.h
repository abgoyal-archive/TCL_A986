
#include "SDL_config.h"

/* Utilities for getting and setting the X display mode */

#include "SDL_cgxvideo.h"

#define GRAB_FULLSCREEN

extern int CGX_GetVideoModes(_THIS);
extern SDL_Rect **CGX_ListModes(_THIS, SDL_PixelFormat *format, Uint32 flags);
extern void CGX_FreeVideoModes(_THIS);
extern int CGX_ResizeFullScreen(_THIS);
extern int CGX_EnterFullScreen(_THIS);
extern int CGX_LeaveFullScreen(_THIS);
