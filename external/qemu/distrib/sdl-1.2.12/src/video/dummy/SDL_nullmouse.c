
#include "SDL_config.h"

#include "SDL_mouse.h"
#include "../../events/SDL_events_c.h"

#include "SDL_nullmouse_c.h"


/* The implementation dependent data for the window manager cursor */
struct WMcursor {
	int unused;
};
