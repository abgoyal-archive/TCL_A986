
#include "SDL_config.h"

#include "SDL_version.h"
#include "SDL_timer.h"
#include "SDL_video.h"
#include "SDL_syswm.h"
#include "../../events/SDL_events_c.h"
#include "../SDL_pixels_c.h"
#include "SDL_cgxmodes_c.h"
#include "SDL_cgxwm_c.h"

/* This is necessary for working properly with Enlightenment, etc. */
#define USE_ICON_WINDOW

void CGX_SetIcon(_THIS, SDL_Surface *icon, Uint8 *mask)
{
/* Not yet implemented */
}

void CGX_SetCaption(_THIS, const char *title, const char *icon)
{
	if(SDL_Window)
		SetWindowTitles(SDL_Window,(char *)title,NULL);
}

/* Iconify the window */
int CGX_IconifyWindow(_THIS)
{
/* Not yet implemented */
	return 0;
}

int CGX_GetWMInfo(_THIS, SDL_SysWMinfo *info)
{
	if ( info->version.major <= SDL_MAJOR_VERSION ) {
		return(1);
	} else {
		SDL_SetError("Application not compiled with SDL %d.%d\n",
					SDL_MAJOR_VERSION, SDL_MINOR_VERSION);
		return(-1);
	}
}
