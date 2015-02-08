
#include "SDL_config.h"

#ifndef __SDL_NTO_AUDIO_H__
#define __SDL_NTO_AUDIO_H__

#include <sys/asoundlib.h>

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the audio functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData
{
    /* The audio device handle */
    int cardno;
    int deviceno;
    snd_pcm_t* audio_handle;

    /* The audio file descriptor */
    int audio_fd;

    /* The parent process id, to detect when application quits */
    pid_t parent;

    /* Raw mixing buffer */
    Uint8* pcm_buf;
    Uint32 pcm_len;

    /* QSA parameters */
    snd_pcm_channel_status_t cstatus;
    snd_pcm_channel_params_t cparams;
    snd_pcm_channel_setup_t  csetup;
};

#define cardno          (this->hidden->cardno)
#define deviceno        (this->hidden->deviceno)
#define audio_handle    (this->hidden->audio_handle)
#define audio_fd        (this->hidden->audio_fd)
#define parent          (this->hidden->parent)
#define pcm_buf         (this->hidden->pcm_buf)
#define pcm_len         (this->hidden->pcm_len)
#define cstatus         (this->hidden->cstatus)
#define cparams         (this->hidden->cparams)
#define csetup          (this->hidden->csetup)

#endif /* __SDL_NTO_AUDIO_H__ */
