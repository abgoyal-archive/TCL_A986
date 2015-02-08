
#include "SDL_config.h"

#ifndef _SDL_lowaudio_h
#define _SDL_lowaudio_h

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *_this

struct SDL_PrivateAudioData {
	BSoundPlayer *audio_obj;
};

/* Old variable names */
#define audio_obj	(_this->hidden->audio_obj)

#endif /* _SDL_lowaudio_h */
