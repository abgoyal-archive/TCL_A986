
#include "SDL_config.h"

#ifndef _SDL_coreaudio_h
#define _SDL_coreaudio_h

#include "../SDL_sysaudio.h"
#include <AudioUnit/AUNTComponent.h>

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	AudioUnit outputAudioUnit;
	void *buffer;
	UInt32 bufferOffset;
	UInt32 bufferSize;
};

/* Old variable names */
#define outputAudioUnit		(this->hidden->outputAudioUnit)
#define buffer		(this->hidden->buffer)
#define bufferOffset		(this->hidden->bufferOffset)
#define bufferSize		(this->hidden->bufferSize)

#endif /* _SDL_coreaudio_h */
