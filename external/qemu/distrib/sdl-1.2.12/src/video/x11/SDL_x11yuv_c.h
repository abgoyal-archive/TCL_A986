
#include "SDL_config.h"

/* This is the XFree86 Xv extension implementation of YUV video overlays */

#include "SDL_video.h"
#include "SDL_x11video.h"

#if SDL_VIDEO_DRIVER_X11_XV

extern SDL_Overlay *X11_CreateYUVOverlay(_THIS, int width, int height, Uint32 format, SDL_Surface *display);

extern int X11_LockYUVOverlay(_THIS, SDL_Overlay *overlay);

extern void X11_UnlockYUVOverlay(_THIS, SDL_Overlay *overlay);

extern int X11_DisplayYUVOverlay(_THIS, SDL_Overlay *overlay, SDL_Rect *src, SDL_Rect *dst);

extern void X11_FreeYUVOverlay(_THIS, SDL_Overlay *overlay);

#endif /* SDL_VIDEO_DRIVER_X11_XV */
