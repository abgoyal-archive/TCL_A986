
#include "SDL_config.h"


#ifndef _SDL_ATARI_BIOSEVENTS_H_
#define _SDL_ATARI_BIOSEVENTS_H_

#include "../SDL_sysvideo.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this

extern void AtariBios_InitOSKeymap(_THIS);
extern void AtariBios_PumpEvents(_THIS);
extern void AtariBios_ShutdownEvents(void);

#endif /* _SDL_ATARI_BIOSEVENTS_H_ */
