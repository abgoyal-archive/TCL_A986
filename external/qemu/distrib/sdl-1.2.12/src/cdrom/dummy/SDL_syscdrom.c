
#include "SDL_config.h"

#if defined(SDL_CDROM_DUMMY) || defined(SDL_CDROM_DISABLED)

/* Stub functions for system-level CD-ROM audio control */

#include "SDL_cdrom.h"
#include "../SDL_syscdrom.h"

int  SDL_SYS_CDInit(void)
{
	return(0);
}

void SDL_SYS_CDQuit(void)
{
	return;
}

#endif /* SDL_CDROM_DUMMY || SDL_CDROM_DISABLED */
