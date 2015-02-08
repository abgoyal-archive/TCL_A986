
#include "SDL_config.h"


/*--- Includes ---*/

#include "SDL_stdinc.h"
#include "SDL_xbios.h"
#include "SDL_xbios_sb3.h"

/*--- Defines ---*/

const int SDL_XBIOS_scpn_planes_device[]={
	SCPN_DEV_1BPP,
	SCPN_DEV_4BPP,
	SCPN_DEV_8BPP,
	SCPN_DEV_16BPP,
	SCPN_DEV_2BPP,
	SCPN_DEV_4BPP,
	SCPN_DEV_1BPP
};

/*--- Functions ---*/

int SDL_XBIOS_SB3Usable(scpn_cookie_t *cookie_scpn)
{
	scpn_screeninfo_t *scrinfo;
	int bpp;

	/* Check if current SB3 mode is usable, i.e. 8 or 16bpp */
	scrinfo = cookie_scpn->screen_info;
	bpp = 1<<(SDL_XBIOS_scpn_planes_device[scrinfo->device]);

	if ((bpp==8) || (bpp==16)) {
		return 1;
	}

	return 0;
}

void SDL_XBIOS_SB3Init(_THIS, scpn_cookie_t *cookie_scpn)
{
	scpn_screeninfo_t *scrinfo;

	/* SB3 prevent changing video modes, we can only use current one */
	if (XBIOS_modelist) {
		SDL_free(XBIOS_modelist);
		XBIOS_nummodes = 0;
		XBIOS_modelist = NULL;
	}

	scrinfo = cookie_scpn->screen_info;
	scrinfo->h_pos = scrinfo->v_pos = 0;

	SDL_XBIOS_AddMode(this,
		-1,
		scrinfo->virtual_width, scrinfo->virtual_height,
		1<<(SDL_XBIOS_scpn_planes_device[scrinfo->device]),
		SDL_FALSE
	);
}
