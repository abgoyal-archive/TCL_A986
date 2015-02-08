
#include "SDL_config.h"

#ifndef _SDL_diskaudio_h
#define _SDL_diskaudio_h

#include "SDL_rwops.h"
#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* The file descriptor for the audio device */
	SDL_RWops *output;
	Uint8 *mixbuf;
	Uint32 mixlen;
	Uint32 write_delay;
};

#endif /* _SDL_diskaudio_h */
