
#include "SDL_config.h"

#ifndef _SDL_artscaudio_h
#define _SDL_artscaudio_h

#include <artsc.h>

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* The stream descriptor for the audio device */
	arts_stream_t stream;

	/* The parent process id, to detect when application quits */
	pid_t parent;

	/* Raw mixing buffer */
	Uint8 *mixbuf;
	int    mixlen;

	/* Support for audio timing using a timer, in addition to select() */
	float frame_ticks;
	float next_frame;
};
#define FUDGE_TICKS	10	/* The scheduler overhead ticks per frame */

/* Old variable names */
#define stream			(this->hidden->stream)
#define parent			(this->hidden->parent)
#define mixbuf			(this->hidden->mixbuf)
#define mixlen			(this->hidden->mixlen)
#define frame_ticks		(this->hidden->frame_ticks)
#define next_frame		(this->hidden->next_frame)

#endif /* _SDL_artscaudio_h */

