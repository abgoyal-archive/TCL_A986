
#include "SDL_config.h"

#ifndef _SDL_UMSaudio_h
#define _SDL_UMSaudio_h

#include <UMS/UMSAudioDevice.h>

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData
{
    /* Pointer to the (open) UMS audio device */
    Environment*   ev;
    UMSAudioDevice umsdev;

    /* Raw mixing buffer */
    UMSAudioTypes_Buffer playbuf;
    UMSAudioTypes_Buffer fillbuf;

    long bytesPerSample;
};

#endif /* _SDL_UMSaudio_h */

