
#include "SDL_config.h"


#ifndef _SDL_ATARI_MXALLOC_H_
#define _SDL_ATARI_MXALLOC_H_

/*--- Defines ---*/

/* Mxalloc parameters */
#define MX_STRAM 0
#define MX_TTRAM 1
#define MX_PREFSTRAM 2
#define MX_PREFTTRAM 3

/*--- Functions ---*/

extern void *Atari_SysMalloc(Uint32 size, Uint16 alloc_type);

#endif /* _SDL_ATARI_MXALLOC_H_ */
