
#include "SDL_config.h"

#ifndef _SDL_dcaudio_h
#define _SDL_dcaudio_h

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* The file descriptor for the audio device */
	Uint8 *mixbuf;
	Uint32 mixlen;
	int playing;
	int leftpos,rightpos;
	int nextbuf;
};

#endif /* _SDL_dcaudio_h */
