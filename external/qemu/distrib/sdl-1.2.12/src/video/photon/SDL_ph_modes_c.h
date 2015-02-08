
#include "SDL_config.h"

#ifndef __SDL_PH_MODES_H__
#define __SDL_PH_MODES_H__

#include "SDL_ph_video.h"

#define PH_MAX_VIDEOMODES 127

#define PH_ENTER_DIRECTMODE  0
#define PH_IGNORE_DIRECTMODE 1

extern SDL_Rect **ph_ListModes(_THIS,SDL_PixelFormat *format, Uint32 flags);
extern void ph_FreeVideoModes(_THIS);
extern int ph_ResizeFullScreen(_THIS);
extern int ph_EnterFullScreen(_THIS, SDL_Surface* screen, int fmode);
extern int ph_LeaveFullScreen(_THIS);
extern int ph_GetVideoMode(int width, int height, int bpp);
extern int get_mode_any_format(int width, int height, int bpp);
extern int ph_ToggleFullScreen(_THIS, int on);

#endif /* __SDL_PH_MODES_H__ */
