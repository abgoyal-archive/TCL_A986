
#include "SDL_config.h"

#ifndef __SDL_PH_WM_H__
#define __SDL_PH_WM_H__

#include "SDL_ph_video.h"

/* Functions to be exported */
extern void ph_SetCaption(_THIS, const char *title, const char *icon);
extern void ph_SetIcon(_THIS, SDL_Surface *icon, Uint8 *mask);
extern int ph_IconifyWindow(_THIS);
extern SDL_GrabMode ph_GrabInputNoLock(_THIS, SDL_GrabMode mode);
extern SDL_GrabMode ph_GrabInput(_THIS, SDL_GrabMode mode);
extern int ph_GetWMInfo(_THIS, SDL_SysWMinfo *info);

#endif /* __SDL_PH_WM_H__ */
