
#include "SDL_config.h"

#ifndef _ALSA_PCM_audio_h
#define _ALSA_PCM_audio_h

#define ALSA_PCM_OLD_HW_PARAMS_API
#define ALSA_PCM_OLD_SW_PARAMS_API
#include <alsa/asoundlib.h>

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	/* The audio device handle */
	snd_pcm_t *pcm_handle;

	/* The parent process id, to detect when application quits */
	pid_t parent;

	/* Raw mixing buffer */
	Uint8 *mixbuf;
	int    mixlen;
};

/* Old variable names */
#define pcm_handle		(this->hidden->pcm_handle)
#define parent			(this->hidden->parent)
#define mixbuf			(this->hidden->mixbuf)
#define mixlen			(this->hidden->mixlen)

#endif /* _ALSA_PCM_audio_h */
