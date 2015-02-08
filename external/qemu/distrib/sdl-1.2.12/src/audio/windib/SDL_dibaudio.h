
#include "SDL_config.h"

#ifndef _SDL_lowaudio_h
#define _SDL_lowaudio_h

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

#define NUM_BUFFERS 2			/* -- Don't lower this! */

struct SDL_PrivateAudioData {
	HWAVEOUT          sound;
	CRITICAL_SECTION  audio_cs;
	HANDLE            audio_event;
	Uint8*            mixbuf;		/* The raw allocated mixing buffer */
	WAVEHDR           wavebuf[NUM_BUFFERS];	/* Wave audio fragments */
	int              cur_buffer;
	int              next_buffer;
};

/* Old variable names */
#define sound			(this->hidden->sound)
#define audio_event		(this->hidden->audio_event)
#define audio_cs        (this->hidden->audio_cs)
#define mixbuf			(this->hidden->mixbuf)
#define wavebuf			(this->hidden->wavebuf)
#define cur_buffer		(this->hidden->cur_buffer)
#define next_buffer		(this->hidden->next_buffer)

#endif /* _SDL_lowaudio_h */
