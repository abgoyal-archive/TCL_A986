
#include "SDL_config.h"

#ifndef _SDL_nasaudio_h
#define _SDL_nasaudio_h

#ifdef __sgi
#include <nas/audiolib.h>
#else
#include <audio/audiolib.h>
#endif
#include <sys/time.h>

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_AudioDevice *this

struct SDL_PrivateAudioData {
	AuServer*  aud;
	AuFlowID   flow;
	AuDeviceID dev;
	
	/* The parent process id, to detect when application quits */
	pid_t parent;

	/* Raw mixing buffer */
	Uint8 *mixbuf;
	int    mixlen;

	int written;
	int really;
	int bps;
	struct timeval last_tv;
	int buf_free;
};
#endif /* _SDL_nasaudio_h */

