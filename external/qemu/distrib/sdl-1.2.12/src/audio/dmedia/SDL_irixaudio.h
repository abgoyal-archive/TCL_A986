
#include "SDL_config.h"

#ifndef _SDL_lowaudio_h
#define _SDL_lowaudio_h

#include <dmedia/audio.h>

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the audio functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* The handle for the audio device */
	ALport audio_port;

	Uint8 *mixbuf;           /* The app mixing buffer */
};

/* Old variable names */
#define audio_port		(this->hidden->audio_port)
#define mixbuf			(this->hidden->mixbuf)

#endif /* _SDL_lowaudio_h */
