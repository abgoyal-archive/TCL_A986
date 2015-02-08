
#include "SDL_config.h"


#ifndef _SDL_ATARI_DEVMOUSE_H_
#define _SDL_ATARI_DEVMOUSE_H_

#include "../SDL_sysvideo.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this

extern int SDL_AtariDevMouse_Open(void);
extern void SDL_AtariDevMouse_Close(void);
extern void SDL_AtariDevMouse_PostMouseEvents(_THIS, SDL_bool buttonEvents);

#endif /* _SDL_ATARI_DEVMOUSE_H_ */
