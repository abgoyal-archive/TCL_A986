
#include "SDL_config.h"


#include "SDL_gemvideo.h"

/* Functions prototypes */
extern void GEM_SetCaption(_THIS, const char *title, const char *icon);
extern void GEM_SetIcon(_THIS, SDL_Surface *icon, Uint8 *mask);
extern int GEM_IconifyWindow(_THIS);
extern SDL_GrabMode GEM_GrabInput(_THIS, SDL_GrabMode mode);
