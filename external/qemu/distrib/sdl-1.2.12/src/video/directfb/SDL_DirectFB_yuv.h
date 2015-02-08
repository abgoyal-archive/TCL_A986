
#include "SDL_config.h"

/* This is the DirectFB implementation of YUV video overlays */

#include "SDL_video.h"
#include "SDL_DirectFB_video.h"

extern SDL_Overlay *DirectFB_CreateYUVOverlay(_THIS, int width, int height, Uint32 format, SDL_Surface *display);

extern int DirectFB_LockYUVOverlay(_THIS, SDL_Overlay *overlay);

extern void DirectFB_UnlockYUVOverlay(_THIS, SDL_Overlay *overlay);

extern int DirectFB_DisplayYUVOverlay(_THIS, SDL_Overlay *overlay, SDL_Rect *src, SDL_Rect *dst);

extern void DirectFB_FreeYUVOverlay(_THIS, SDL_Overlay *overlay);

