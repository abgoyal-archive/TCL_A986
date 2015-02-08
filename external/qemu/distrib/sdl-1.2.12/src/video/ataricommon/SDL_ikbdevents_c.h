
#include "SDL_config.h"


#ifndef _SDL_ATARI_IKBDEVENTS_H_
#define _SDL_ATARI_IKBDEVENTS_H_

#include "../SDL_sysvideo.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this

extern void AtariIkbd_InitOSKeymap(_THIS);
extern void AtariIkbd_PumpEvents(_THIS);
extern void AtariIkbd_ShutdownEvents(void);

#endif /* _SDL_ATARI_IKBDEVENTS_H_ */
