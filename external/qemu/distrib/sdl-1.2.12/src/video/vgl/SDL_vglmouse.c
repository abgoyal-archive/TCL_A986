
#include "SDL_config.h"

#include "SDL_mouse.h"
#include "../../events/SDL_events_c.h"
#include "SDL_vglvideo.h"
#include "SDL_vglmouse_c.h"


/* The implementation dependent data for the window manager cursor */
struct WMcursor {
	int unused;
};


void VGL_FreeWMCursor(_THIS, WMcursor *cursor)
{
	return;
}

WMcursor *VGL_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y)
{
	return(NULL);
}

int VGL_ShowWMCursor(_THIS, WMcursor *cursor)
{
	return(0);
}

void VGL_WarpWMCursor(_THIS, Uint16 x, Uint16 y)
{
	SDL_PrivateMouseMotion(0, 0, x, y);
}

