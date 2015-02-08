
#include "SDL_config.h"

/* This is the DirectDraw implementation of YUV video overlays */

#include "SDL_video.h"
#include "../wincommon/SDL_lowvideo.h"
#include "SDL_dx5video.h"

extern SDL_Overlay *DX5_CreateYUVOverlay(_THIS, int width, int height, Uint32 format, SDL_Surface *display);

extern int DX5_LockYUVOverlay(_THIS, SDL_Overlay *overlay);

extern void DX5_UnlockYUVOverlay(_THIS, SDL_Overlay *overlay);

extern int DX5_DisplayYUVOverlay(_THIS, SDL_Overlay *overlay, SDL_Rect *src, SDL_Rect *dst);

extern void DX5_FreeYUVOverlay(_THIS, SDL_Overlay *overlay);
