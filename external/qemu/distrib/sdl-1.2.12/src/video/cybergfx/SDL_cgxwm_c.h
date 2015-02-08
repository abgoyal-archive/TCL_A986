
#include "SDL_config.h"

#include "SDL_cgxvideo.h"

/* Functions to be exported */
extern void CGX_SetCaption(_THIS, const char *title, const char *icon);
extern void CGX_SetIcon(_THIS, SDL_Surface *icon, Uint8 *mask);
extern int CGX_IconifyWindow(_THIS);
extern SDL_GrabMode CGX_GrabInputNoLock(_THIS, SDL_GrabMode mode);
extern SDL_GrabMode CGX_GrabInput(_THIS, SDL_GrabMode mode);
extern int CGX_GetWMInfo(_THIS, SDL_SysWMinfo *info);

