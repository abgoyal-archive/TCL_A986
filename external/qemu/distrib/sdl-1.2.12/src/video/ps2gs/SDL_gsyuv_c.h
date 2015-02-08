
#include "SDL_config.h"

/* This is the Playstation 2 implementation of YUV video overlays */

#include "SDL_video.h"
#include "SDL_gsvideo.h"

extern SDL_Overlay *GS_CreateYUVOverlay(_THIS, int width, int height, Uint32 format, SDL_Surface *display);

extern int GS_LockYUVOverlay(_THIS, SDL_Overlay *overlay);

extern void GS_UnlockYUVOverlay(_THIS, SDL_Overlay *overlay);

extern int GS_DisplayYUVOverlay(_THIS, SDL_Overlay *overlay, SDL_Rect *src, SDL_Rect *dst);

extern void GS_FreeYUVOverlay(_THIS, SDL_Overlay *overlay);
