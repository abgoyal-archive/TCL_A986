
#include "SDL_config.h"

#ifndef _SDL_lowaudio_h
#define _SDL_lowaudio_h

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* The file descriptor for the audio device */
	int audio_fd;

	Uint16 audio_fmt;        /* The app audio format */
	Uint8 *mixbuf;           /* The app mixing buffer */
	int ulaw_only;           /* Flag -- does hardware only output U-law? */
	Uint8 *ulaw_buf;         /* The U-law mixing buffer */
	Sint32 written;          /* The number of samples written */
	int fragsize;            /* The audio fragment size in samples */
	int frequency;           /* The audio frequency in KHz */
};

/* Old variable names */
#define audio_fd		(this->hidden->audio_fd)
#define audio_fmt		(this->hidden->audio_fmt)
#define mixbuf			(this->hidden->mixbuf)
#define ulaw_only		(this->hidden->ulaw_only)
#define ulaw_buf		(this->hidden->ulaw_buf)
#define written			(this->hidden->written)
#define fragsize		(this->hidden->fragsize)
#define frequency		(this->hidden->frequency)

#endif /* _SDL_lowaudio_h */
