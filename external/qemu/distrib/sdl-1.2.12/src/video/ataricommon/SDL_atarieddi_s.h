
#include "SDL_config.h"

#ifndef _SDL_Atari_eddi_s_h
#define _SDL_Atari_eddi_s_h

/*--- Defines ---*/

/* EdDI versions */

#define EDDI_10	(0x0100)
#define EDDI_11 (0x0110)

/* Screen format */

enum {
	VDI_FORMAT_UNKNOWN=-1,
	VDI_FORMAT_INTER=0,	/* Interleaved bitplanes */
	VDI_FORMAT_VDI=1,	/* VDI independent */
	VDI_FORMAT_PACK=2	/* Packed pixels */
};

/* CLUT types */
enum {
	VDI_CLUT_NONE=0,	/* Monochrome mode */
	VDI_CLUT_HARDWARE,	/* <256 colours mode */
	VDI_CLUT_SOFTWARE	/* True colour mode */
};

/*--- Functions ---*/

unsigned long Atari_get_EdDI_version(void *function_pointer);

#endif /* _SDL_Atari_eddi_s_h */
