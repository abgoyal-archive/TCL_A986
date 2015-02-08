
#include "SDL_config.h"

/* Allow access to a raw mixing buffer */

#ifndef _SDL_lowaudio_h
#define _SDL_lowaudio_h

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this
#define NUM_BUFFERS 2

struct SharedMem {
    HWAVEOUT sound;
    WAVEHDR wHdr[NUM_BUFFERS];
    PCMWAVEFORMAT wFmt;
};

struct SDL_PrivateAudioData {
    Uint8 *mixbuf;          /* The raw allocated mixing buffer */
    struct SharedMem *shm;
    int next_buffer;
};

#define shm			(this->hidden->shm)
#define mixbuf			(this->hidden->mixbuf)
#define next_buffer		(this->hidden->next_buffer)
/* Old variable names */
#endif /* _SDL_lowaudio_h */
