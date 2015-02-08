
#include "SDL_config.h"


#ifndef __SDL_SYS_YUV_H__
#define __SDL_SYS_YUV_H__

/* This is the BeOS implementation of YUV video overlays */

#include "SDL_video.h"
#include "SDL_lowvideo.h"

extern "C" {

struct private_yuvhwdata
{
	SDL_Surface *display;
	BView *bview;
	bool first_display;
	BBitmap *bbitmap;
    int locked;
};

extern BBitmap * BE_GetOverlayBitmap(BRect bounds, color_space cs);
extern SDL_Overlay* BE_CreateYUVOverlay(_THIS, int width, int height, Uint32 format, SDL_Surface* display);
extern int BE_LockYUVOverlay(_THIS, SDL_Overlay* overlay);
extern void BE_UnlockYUVOverlay(_THIS, SDL_Overlay* overlay);
extern int BE_DisplayYUVOverlay(_THIS, SDL_Overlay* overlay, SDL_Rect* src, SDL_Rect* dst);
extern void BE_FreeYUVOverlay(_THIS, SDL_Overlay* overlay);

};

#endif /* __SDL_PH_YUV_H__ */
