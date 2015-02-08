
#include "SDL_config.h"

#ifndef _SDL_dummyaudio_h
#define _SDL_dummyaudio_h

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* The file descriptor for the audio device */
	Uint8 *mixbuf;
	Uint32 mixlen;
	Uint32 write_delay;
	Uint32 initial_calls;
};

#endif /* _SDL_dummyaudio_h */
