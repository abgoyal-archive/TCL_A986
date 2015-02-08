

/* Access to the raw audio mixing buffer for the SDL library */

#ifndef _SDL_audio_h
#define _SDL_audio_h

#include "SDL_stdinc.h"
#include "SDL_error.h"
#include "SDL_endian.h"
#include "SDL_mutex.h"
#include "SDL_thread.h"
#include "SDL_rwops.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

/* The calculated values in this structure are calculated by SDL_OpenAudio() */
typedef struct SDL_AudioSpec {
	int freq;		/* DSP frequency -- samples per second */
	Uint16 format;		/* Audio data format */
	Uint8  channels;	/* Number of channels: 1 mono, 2 stereo */
	Uint8  silence;		/* Audio buffer silence value (calculated) */
	Uint16 samples;		/* Audio buffer size in samples (power of 2) */
	Uint16 padding;		/* Necessary for some compile environments */
	Uint32 size;		/* Audio buffer size in bytes (calculated) */
	/* This function is called when the audio device needs more data.
	   'stream' is a pointer to the audio data buffer
	   'len' is the length of that buffer in bytes.
	   Once the callback returns, the buffer will no longer be valid.
	   Stereo samples are stored in a LRLRLR ordering.
	*/
	void (SDLCALL *callback)(void *userdata, Uint8 *stream, int len);
	void  *userdata;
} SDL_AudioSpec;

/* Audio format flags (defaults to LSB byte order) */
#define AUDIO_U8	0x0008	/* Unsigned 8-bit samples */
#define AUDIO_S8	0x8008	/* Signed 8-bit samples */
#define AUDIO_U16LSB	0x0010	/* Unsigned 16-bit samples */
#define AUDIO_S16LSB	0x8010	/* Signed 16-bit samples */
#define AUDIO_U16MSB	0x1010	/* As above, but big-endian byte order */
#define AUDIO_S16MSB	0x9010	/* As above, but big-endian byte order */
#define AUDIO_U16	AUDIO_U16LSB
#define AUDIO_S16	AUDIO_S16LSB

/* Native audio byte ordering */
#if SDL_BYTEORDER == SDL_LIL_ENDIAN
#define AUDIO_U16SYS	AUDIO_U16LSB
#define AUDIO_S16SYS	AUDIO_S16LSB
#else
#define AUDIO_U16SYS	AUDIO_U16MSB
#define AUDIO_S16SYS	AUDIO_S16MSB
#endif


/* A structure to hold a set of audio conversion filters and buffers */
typedef struct SDL_AudioCVT {
	int needed;			/* Set to 1 if conversion possible */
	Uint16 src_format;		/* Source audio format */
	Uint16 dst_format;		/* Target audio format */
	double rate_incr;		/* Rate conversion increment */
	Uint8 *buf;			/* Buffer to hold entire audio data */
	int    len;			/* Length of original audio buffer */
	int    len_cvt;			/* Length of converted audio buffer */
	int    len_mult;		/* buffer must be len*len_mult big */
	double len_ratio; 	/* Given len, final size is len*len_ratio */
	void (SDLCALL *filters[10])(struct SDL_AudioCVT *cvt, Uint16 format);
	int filter_index;		/* Current audio conversion function */
} SDL_AudioCVT;


/* Function prototypes */

extern DECLSPEC int SDLCALL SDL_AudioInit(const char *driver_name);
extern DECLSPEC void SDLCALL SDL_AudioQuit(void);

extern DECLSPEC char * SDLCALL SDL_AudioDriverName(char *namebuf, int maxlen);

extern DECLSPEC int SDLCALL SDL_OpenAudio(SDL_AudioSpec *desired, SDL_AudioSpec *obtained);

typedef enum {
	SDL_AUDIO_STOPPED = 0,
	SDL_AUDIO_PLAYING,
	SDL_AUDIO_PAUSED
} SDL_audiostatus;
extern DECLSPEC SDL_audiostatus SDLCALL SDL_GetAudioStatus(void);

extern DECLSPEC void SDLCALL SDL_PauseAudio(int pause_on);

extern DECLSPEC SDL_AudioSpec * SDLCALL SDL_LoadWAV_RW(SDL_RWops *src, int freesrc, SDL_AudioSpec *spec, Uint8 **audio_buf, Uint32 *audio_len);

/* Compatibility convenience function -- loads a WAV from a file */
#define SDL_LoadWAV(file, spec, audio_buf, audio_len) \
	SDL_LoadWAV_RW(SDL_RWFromFile(file, "rb"),1, spec,audio_buf,audio_len)

extern DECLSPEC void SDLCALL SDL_FreeWAV(Uint8 *audio_buf);

extern DECLSPEC int SDLCALL SDL_BuildAudioCVT(SDL_AudioCVT *cvt,
		Uint16 src_format, Uint8 src_channels, int src_rate,
		Uint16 dst_format, Uint8 dst_channels, int dst_rate);

extern DECLSPEC int SDLCALL SDL_ConvertAudio(SDL_AudioCVT *cvt);

#define SDL_MIX_MAXVOLUME 128
extern DECLSPEC void SDLCALL SDL_MixAudio(Uint8 *dst, const Uint8 *src, Uint32 len, int volume);

extern DECLSPEC void SDLCALL SDL_LockAudio(void);
extern DECLSPEC void SDLCALL SDL_UnlockAudio(void);

extern DECLSPEC void SDLCALL SDL_CloseAudio(void);


/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_audio_h */
