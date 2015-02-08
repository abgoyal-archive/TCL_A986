
#include "SDL_config.h"

#ifndef _SDL_romaudio_h
#define _SDL_romaudio_h

#include "../SDL_sysaudio.h"

/* This is Ryan's improved MacOS sound code, with locking support */
#define USE_RYANS_SOUNDCODE

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* Sound manager audio channel */
	SndChannelPtr channel;
#if defined(TARGET_API_MAC_CARBON) || defined(USE_RYANS_SOUNDCODE)
	/* FIXME: Add Ryan's static data here */
#else
	/* Double buffering variables */
	SndDoubleBufferPtr audio_buf[2];
#endif
};

/* Old variable names */
#define channel		(this->hidden->channel)
#define audio_buf	(this->hidden->audio_buf)

#endif /* _SDL_romaudio_h */
