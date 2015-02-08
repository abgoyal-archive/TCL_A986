
#include "SDL_config.h"


#include <mint/falcon.h>

#include "SDL_xbios.h"
#include "SDL_xbios_blowup.h"

void SDL_XBIOS_BlowupInit(_THIS, blow_cookie_t *cookie_blow)
{
	int i, num_mode, bank;
	blow_mode_t *blow_mode;

	/* Add bit 15 for old modes */
	for (i=0;i<XBIOS_nummodes;i++) {
		XBIOS_modelist[i].number |= 1<<15;
	}

	/* Add Blowup modes for 8 and 16 bpp */
	for (num_mode=3; num_mode<5; num_mode++) {
		bank = cookie_blow->num_mode[num_mode];
		blow_mode = &(cookie_blow->blowup_modes[num_mode+(bank*5)]);

		/* Check extended mode enabled */
		if (blow_mode->enabled == 0) {
			/* Check monitor needed for this mode */
			if ((blow_mode->monitor == cookie_blow->montype)
				|| ((blow_mode->monitor == MONITOR_TV)
					&& (cookie_blow->montype == MONITOR_RGB))
				|| ((blow_mode->monitor == MONITOR_RGB)
					&& (cookie_blow->montype == MONITOR_TV)))
			{
				/* we can use this extended mode */
				SDL_XBIOS_AddMode(this,
					num_mode == 3 ? BPS8 : BPS16,
					blow_mode->width + 1,
					blow_mode->height + 1,
					num_mode == 3 ? 8 : 16,
					SDL_FALSE
				);
			}
		}
	}
}
