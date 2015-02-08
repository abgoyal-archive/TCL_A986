
#include "SDL_config.h"

#ifndef _SDL_lowaudio_h
#define _SDL_lowaudio_h

#include "directx.h"

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

/* The DirectSound objects */
struct SDL_PrivateAudioData {
	LPDIRECTSOUND sound;
	LPDIRECTSOUNDBUFFER mixbuf;
	int NUM_BUFFERS;
	int mixlen, silence;
	DWORD lastchunk;
	Uint8 *locked_buf;
	HANDLE audio_event;
};

/* Old variable names */
#define sound			(this->hidden->sound)
#define mixbuf			(this->hidden->mixbuf)
#define NUM_BUFFERS		(this->hidden->NUM_BUFFERS)
#define mixlen			(this->hidden->mixlen)
#define silence			(this->hidden->silence)
#define lastchunk		(this->hidden->lastchunk)
#define locked_buf		(this->hidden->locked_buf)
#define audio_event		(this->hidden->audio_event)

#endif /* _SDL_lowaudio_h */
