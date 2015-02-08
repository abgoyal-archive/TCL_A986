
#include "SDL_config.h"


#ifndef _SDL_ATARI_EVENTS_H_
#define _SDL_ATARI_EVENTS_H_

#include "../SDL_sysvideo.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this

#define ATARIBIOS_MAXKEYS 128

/* Special keys state */
#ifndef K_RSHIFT
enum {
	K_RSHIFT=0,
	K_LSHIFT,
	K_CTRL,
	K_ALT,
	K_CAPSLOCK,
	K_CLRHOME,
	K_INSERT
};
#endif

extern void (*Atari_ShutdownEvents)(void);

extern void Atari_InitOSKeymap(_THIS);
extern void Atari_PumpEvents(_THIS);

extern void SDL_Atari_InitInternalKeymap(_THIS);

/* Atari to Unicode charset translation table */
extern Uint16 SDL_AtariToUnicodeTable[256];
SDL_keysym *SDL_Atari_TranslateKey(int scancode, SDL_keysym *keysym,
	SDL_bool pressed);

#endif /* _SDL_ATARI_EVENTS_H_ */
