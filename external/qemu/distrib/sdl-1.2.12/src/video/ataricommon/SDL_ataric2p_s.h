
#include "SDL_config.h"

#ifndef _ATARI_C2P_h
#define _ATARI_C2P_h

#include "SDL_stdinc.h"

/*--- Functions pointers ---*/

/* Convert a chunky screen to bitplane screen */

extern void (*SDL_Atari_C2pConvert)(
	Uint8 *src,			/* Source screen (one byte=one pixel) */
	Uint8 *dest,		/* Destination (4/8 bits planes) */
	Uint32 width,		/* Dimensions of screen to convert */
	Uint32 height,
	Uint32 dblligne,	/* Double the lines when converting ? */
	Uint32 srcpitch,	/* Length of one source line in bytes */
	Uint32 dstpitch		/* Length of one destination line in bytes */
);

/*--- 8 bits functions ---*/

/* Convert a chunky screen to bitplane screen */

void SDL_Atari_C2pConvert8(
	Uint8 *src,			/* Source screen (one byte=one pixel) */
	Uint8 *dest,		/* Destination (8 bits planes) */
	Uint32 width,		/* Dimensions of screen to convert */
	Uint32 height,
	Uint32 dblligne,	/* Double the lines when converting ? */
	Uint32 srcpitch,	/* Length of one source line in bytes */
	Uint32 dstpitch		/* Length of one destination line in bytes */
);

/*--- 4 bits functions ---*/

/* Convert a chunky screen to bitplane screen */

void SDL_Atari_C2pConvert4(
	Uint8 *src,			/* Source screen (one byte=one pixel) */
	Uint8 *dest,		/* Destination (4 bits planes) */
	Uint32 width,		/* Dimensions of screen to convert */
	Uint32 height,
	Uint32 dblligne,	/* Double the lines when converting ? */
	Uint32 srcpitch,	/* Length of one source line in bytes */
	Uint32 dstpitch		/* Length of one destination line in bytes */
);

/* Conversion palette */

void SDL_Atari_C2pConvert4_pal(Uint16 *lightpalette);

#endif /* _ATARI_C2P_h */
