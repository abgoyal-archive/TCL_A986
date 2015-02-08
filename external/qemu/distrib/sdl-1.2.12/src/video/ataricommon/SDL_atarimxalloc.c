
#include "SDL_config.h"


#include <mint/osbind.h>

#include "SDL_stdinc.h"

/*--- Variables ---*/

static int atari_mxalloc_avail=-1;

/*--- Functions ---*/

void *Atari_SysMalloc(Uint32 size, Uint16 alloc_type)
{
	/* Test if Mxalloc() available */
	if (atari_mxalloc_avail<0) {
		atari_mxalloc_avail = ((Sversion()&0xFF)>=0x01) | (Sversion()>=0x1900);
	}

	if (atari_mxalloc_avail) {
		return (void *) Mxalloc(size, alloc_type);
	} else { \
		return (void *) Malloc(size);
	}
}
