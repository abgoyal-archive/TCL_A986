
#include "SDL_config.h"

/* Utilities for getting and setting the X display mode */

#include "SDL_x11video.h"

#define GRAB_FULLSCREEN

extern int X11_GetVideoModes(_THIS);
extern SDL_Rect **X11_ListModes(_THIS, SDL_PixelFormat *format, Uint32 flags);
extern void X11_FreeVideoModes(_THIS);
extern int X11_ResizeFullScreen(_THIS);
extern void X11_WaitMapped(_THIS, Window win);
extern void X11_WaitUnmapped(_THIS, Window win);
extern void X11_QueueEnterFullScreen(_THIS);
extern int X11_EnterFullScreen(_THIS);
extern int X11_LeaveFullScreen(_THIS);
