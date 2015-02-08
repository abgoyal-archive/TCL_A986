
#include "SDL_config.h"

#ifndef _SDL_ahiaudio_h
#define _SDL_ahiaudio_h

#include <exec/exec.h>
#include <dos/dos.h>
#ifdef __SASC
#include <proto/exec.h>
#else
#include <inline/exec.h>
#endif

#include <devices/ahi.h>
#include "mydebug.h"

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the audio functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* The handle for the audio device */
	struct AHIRequest *audio_req[2];
	struct MsgPort *audio_port;
	Sint32 freq,type,bytespersample,size;
	Uint8 *mixbuf[2];           /* The app mixing buffer */
	int current_buffer;
	Uint32 playing;
};

/* Old variable names */
#define audio_port		(this->hidden->audio_port)
#define audio_req		(this->hidden->audio_req)
#define mixbuf			(this->hidden->mixbuf)
#define current_buffer		(this->hidden->current_buffer)
#define playing			(this->hidden->playing)

#endif /* _SDL_ahiaudio_h */
