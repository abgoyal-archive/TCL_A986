
#include "SDL_config.h"


#ifndef _SDL_mintaudio_mcsh_h
#define _SDL_mintaudio_mcsh_h

typedef struct {
	unsigned short version;	/* Version */
	unsigned short size;	/* Size of structure */

	unsigned short play;	/* Replay capability */
	unsigned short record;	/* Record capability */
	unsigned short dsp;		/* DSP56K present */
	unsigned short pint;	/* Interrupt at end of replay */
	unsigned short rint;	/* Interrupt at end of record */

	unsigned long res1;		/* Frequency of external clock */
	unsigned long res2;
	unsigned long res3;
	unsigned long res4;
} cookie_mcsn_t __attribute__((packed));

enum {
	MCSN_ST=0,
	MCSN_TT,
	MCSN_STE=MCSN_TT,
	MCSN_FALCON,
	MCSN_MAC=MCSN_FALCON
};

#define SETSMPFREQ	7	/* Set sample frequency */

#endif /* _SDL_mintaudio_mcsh_h */
