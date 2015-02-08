
#include "SDL_config.h"


#ifndef _SDL_ATARI_GEMDOSEVENTS_H_
#define _SDL_ATARI_GEMDOSEVENTS_H_

#include "../SDL_sysvideo.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this

extern void AtariGemdos_InitOSKeymap(_THIS);
extern void AtariGemdos_PumpEvents(_THIS);
extern void AtariGemdos_ShutdownEvents(void);

#endif /* _SDL_ATARI_GEMDOSEVENTS_H_ */
