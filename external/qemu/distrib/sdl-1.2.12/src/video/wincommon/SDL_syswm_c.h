
#include "SDL_config.h"

#include "SDL_lowvideo.h"

/* Data that needs to be freed at SDL_SYS_VideoQuit() */
extern HICON screen_icn;

/* Functions to be exported */
extern void WIN_SetWMIcon(_THIS, SDL_Surface *icon, Uint8 *mask);
extern void WIN_SetWMCaption(_THIS, const char *title, const char *icon);
extern int WIN_IconifyWindow(_THIS);
extern SDL_GrabMode WIN_GrabInput(_THIS, SDL_GrabMode mode);
extern int WIN_GetWMInfo(_THIS, SDL_SysWMinfo *info);

