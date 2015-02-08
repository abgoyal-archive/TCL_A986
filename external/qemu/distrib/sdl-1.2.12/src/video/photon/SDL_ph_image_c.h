
#include "SDL_config.h"

#ifndef __SDL_PH_IMAGE_H__
#define __SDL_PH_IMAGE_H__

#include "../../events/SDL_events_c.h"
#include "SDL_ph_video.h"

struct private_hwdata
{
   PdOffscreenContext_t* offscreenctx;
   PdOSCCreateLockParams_t crlockparam;
   PdOSCLockParams_t lockparam;
   Uint32 colorkey;
};

extern int ph_SetupImage(_THIS, SDL_Surface* screen);
extern void ph_DestroyImage(_THIS, SDL_Surface* screen);
extern int ph_SetupUpdateFunction(_THIS, SDL_Surface* screen, Uint32 flags);

extern int ph_AllocHWSurface(_THIS, SDL_Surface* surface);
extern void ph_FreeHWSurface(_THIS, SDL_Surface* surface);
extern int ph_CheckHWBlit(_THIS, SDL_Surface *src, SDL_Surface *dst);
extern int ph_FillHWRect(_THIS, SDL_Surface* surface, SDL_Rect* rect, Uint32 color);
extern int ph_LockHWSurface(_THIS, SDL_Surface* surface);
extern void ph_UnlockHWSurface(_THIS, SDL_Surface* surface);
extern int ph_FlipHWSurface(_THIS, SDL_Surface* surface);
extern int ph_SetHWColorKey(_THIS, SDL_Surface* surface, Uint32 key);
extern int ph_SetHWAlpha(_THIS, SDL_Surface* surface, Uint8 alpha);
extern int ph_HWAccelBlit(SDL_Surface* src, SDL_Rect *srcrect, SDL_Surface* dst, SDL_Rect* dstrect);
extern int ph_UpdateHWInfo(_THIS);

extern void ph_NormalUpdate(_THIS, int numrects, SDL_Rect* rects);
extern void ph_OCUpdate(_THIS, int numrects, SDL_Rect* rects);
extern void ph_OCDCUpdate(_THIS, int numrects, SDL_Rect* rects);
extern void ph_OpenGLUpdate(_THIS, int numrects, SDL_Rect* rects);

#endif /* __SDL_PH_IMAGE_H__ */
