
#include "SDL_config.h"


#include <mint/falcon.h>

#include "SDL_xbios.h"
#include "SDL_xbios_centscreen.h"

int SDL_XBIOS_CentscreenInit(_THIS)
{
	centscreen_mode_t	curmode, listedmode;
	unsigned long result;
	int cur_handle;	/* Current Centscreen mode handle */

	/* Reset current mode list */
	if (XBIOS_modelist) {
		SDL_free(XBIOS_modelist);
		XBIOS_nummodes = 0;
		XBIOS_modelist = NULL;
	}

	/* Add Centscreen modes */
	Vread(&curmode);
	cur_handle = curmode.handle;
	curmode.mode = curmode.physx = curmode.physy = curmode.plan =
		curmode.logx = curmode.logy = -1;

	result = Vfirst(&curmode, &listedmode);
	if (result==0) {
		while (result==0) {
			/* Don't add modes with virtual screen */
			if ((listedmode.mode & CSCREEN_VIRTUAL)==0) {
				/* Don't add modes with bpp<8 */
				if (listedmode.plan>=8) {
					SDL_XBIOS_AddMode(this, listedmode.mode, listedmode.physx,
						listedmode.physy, listedmode.plan, SDL_FALSE
					);
				}
			}
			SDL_memcpy(&curmode, &listedmode, sizeof(centscreen_mode_t));
			curmode.mode = curmode.physx = curmode.physy = curmode.plan =
				curmode.logx = curmode.logy = -1;
			result = Vnext(&curmode, &listedmode);
		}		
	} else {
		fprintf(stderr, "No suitable Centscreen modes\n");
	}

	return cur_handle;
}

void SDL_XBIOS_CentscreenSetmode(_THIS, int width, int height, int planes)
{
	centscreen_mode_t	newmode, curmode;
	
	newmode.handle = newmode.mode = newmode.logx = newmode.logy = -1;
	newmode.physx = width;
	newmode.physy = height;
	newmode.plan = planes;
	Vwrite(0, &newmode, &curmode);

	/* Disable screensaver */
	Vread(&newmode);
	newmode.mode &= ~(CSCREEN_SAVER|CSCREEN_ENERGYSTAR);
	Vwrite(0, &newmode, &curmode);
}

void SDL_XBIOS_CentscreenRestore(_THIS, int prev_handle)
{
	centscreen_mode_t	newmode, curmode;

	/* Restore old video mode */
	newmode.handle = prev_handle;
	newmode.mode = newmode.physx = newmode.physy = newmode.plan =
		newmode.logx = newmode.logy = -1;
	Vwrite(0, &newmode, &curmode);
}
