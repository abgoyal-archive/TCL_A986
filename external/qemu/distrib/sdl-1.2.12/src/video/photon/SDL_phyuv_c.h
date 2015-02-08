
#include "SDL_config.h"

#ifndef __SDL_PH_YUV_H__
#define __SDL_PH_YUV_H__

/* This is the photon implementation of YUV video overlays */

#include "SDL_video.h"
#include "SDL_ph_video.h"

struct private_yuvhwdata
{
    FRAMEDATA* CurrentFrameData;
    FRAMEDATA* FrameData0;
    FRAMEDATA* FrameData1;
    PgScalerProps_t   props;
    PgScalerCaps_t    caps;
    PgVideoChannel_t* channel;
    PhArea_t CurrentViewPort;
    PhPoint_t CurrentWindowPos;
    long format;
    int scaler_on;
    int current;
    long YStride;
    long VStride;
    long UStride;
    int ischromakey;
    long chromakey;
    int forcedredraw;
    unsigned long State;
    long flags;
    int locked;
};

extern SDL_Overlay* ph_CreateYUVOverlay(_THIS, int width, int height, Uint32 format, SDL_Surface* display);
extern int ph_LockYUVOverlay(_THIS, SDL_Overlay* overlay);
extern void ph_UnlockYUVOverlay(_THIS, SDL_Overlay* overlay);
extern int ph_DisplayYUVOverlay(_THIS, SDL_Overlay* overlay, SDL_Rect* src, SDL_Rect* dst);
extern void ph_FreeYUVOverlay(_THIS, SDL_Overlay* overlay);

#endif /* __SDL_PH_YUV_H__ */
