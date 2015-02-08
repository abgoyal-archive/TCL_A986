
#include "SDL_config.h"


#include "SDL.h"
#include "../../events/SDL_sysevents.h"
#include "../../events/SDL_events_c.h"

#include "SDL_nullvideo.h"
#include "SDL_nullevents_c.h"

void DUMMY_PumpEvents(_THIS)
{
	/* do nothing. */
}

void DUMMY_InitOSKeymap(_THIS)
{
	/* do nothing. */
}

/* end of SDL_nullevents.c ... */

