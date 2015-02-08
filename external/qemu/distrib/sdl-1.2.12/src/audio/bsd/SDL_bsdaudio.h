
#include "SDL_config.h"

#ifndef _SDL_openbsdaudio_h
#define _SDL_openbsdaudio_h

#include "../SDL_sysaudio.h"

#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData
{
    /* The file descriptor for the audio device */
    int audio_fd;

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
#define audio_fd		(this->hidden->audio_fd)
#define parent			(this->hidden->parent)
#define mixbuf			(this->hidden->mixbuf)
#define mixlen			(this->hidden->mixlen)
#define frame_ticks		(this->hidden->frame_ticks)
#define next_frame		(this->hidden->next_frame)

#endif /* _SDL_openbsdaudio_h */
