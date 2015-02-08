
#include "SDL_config.h"


#ifndef _SDL_ATARI_XBIOSEVENTS_H_
#define _SDL_ATARI_XBIOSEVENTS_H_

#include "../SDL_sysvideo.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this

#define ATARI_XBIOS_MOUSEEVENTS (1<<0)
#define ATARI_XBIOS_JOYSTICKEVENTS (1<<1)

extern int SDL_AtariXbios_enabled;

extern void SDL_AtariXbios_InstallVectors(int vectors_mask);
extern void SDL_AtariXbios_RestoreVectors(void);
extern void SDL_AtariXbios_PostMouseEvents(_THIS, SDL_bool buttonEvents);
extern void SDL_AtariXbios_LockMousePosition(SDL_bool lockPosition);

#endif /* _SDL_XBIOSEVENTS_H_ */
