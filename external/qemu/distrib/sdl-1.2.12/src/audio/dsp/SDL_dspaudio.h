
#include "SDL_config.h"

#ifndef _SDL_dspaudio_h
#define _SDL_dspaudio_h

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* The file descriptor for the audio device */
	int audio_fd;

	/* The parent process id, to detect when application quits */
	pid_t parent;

	/* Raw mixing buffer */
	Uint8 *mixbuf;
	int    mixlen;
};
#define FUDGE_TICKS	10	/* The scheduler overhead ticks per frame */

/* Old variable names */
#define audio_fd		(this->hidden->audio_fd)
#define parent			(this->hidden->parent)
#define mixbuf			(this->hidden->mixbuf)
#define mixlen			(this->hidden->mixlen)
#define frame_ticks		(this->hidden->frame_ticks)
#define next_frame		(this->hidden->next_frame)

#endif /* _SDL_dspaudio_h */
