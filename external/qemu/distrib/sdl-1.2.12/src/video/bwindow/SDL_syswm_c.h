
#include "SDL_config.h"

#include "SDL_syswm.h"
#include "SDL_lowvideo.h"


/* Functions to be exported */
extern void BE_SetWMCaption(_THIS, const char *title, const char *icon);
extern int BE_IconifyWindow(_THIS);
extern int BE_GetWMInfo(_THIS, SDL_SysWMinfo *info);
extern SDL_GrabMode BE_GrabInput(_THIS, SDL_GrabMode mode);