
#include "SDL_config.h"


/* Mint includes */
#include <mint/osbind.h>

#include "../../events/SDL_sysevents.h"
#include "../../events/SDL_events_c.h"

#include "SDL_atarikeys.h"
#include "SDL_atarievents_c.h"
#include "SDL_ikbdinterrupt_s.h"

#define KEY_PRESSED		0xff
#define KEY_UNDEFINED	0x80
#define KEY_RELEASED	0x00

static Uint16 atari_prevmouseb;	/* save state of mouse buttons */

void AtariIkbd_InitOSKeymap(_THIS)
{
	int i;

	SDL_memset(SDL_AtariIkbd_keyboard, KEY_UNDEFINED, sizeof(SDL_AtariIkbd_keyboard));

	/* Now install our handler */
	SDL_AtariIkbd_mouseb = SDL_AtariIkbd_mousex = SDL_AtariIkbd_mousey = 0;
	atari_prevmouseb = 0;

	Supexec(SDL_AtariIkbdInstall);
}

static int atari_GetButton(int button)
{
	switch(button)
	{
		case 0:
			return SDL_BUTTON_RIGHT;
			break;
		case 1:
		default:
			return SDL_BUTTON_LEFT;
			break;
	}
}

void AtariIkbd_PumpEvents(_THIS)
{
	int i, specialkeys;
	SDL_keysym keysym;

	/*--- Send keyboard events ---*/

	for (i=0; i<ATARIBIOS_MAXKEYS; i++) {
		/* Key pressed ? */
		if (SDL_AtariIkbd_keyboard[i]==KEY_PRESSED) {
			SDL_PrivateKeyboard(SDL_PRESSED,
				SDL_Atari_TranslateKey(i, &keysym, SDL_TRUE));
			SDL_AtariIkbd_keyboard[i]=KEY_UNDEFINED;
		}
			
		/* Key released ? */
		if (SDL_AtariIkbd_keyboard[i]==KEY_RELEASED) {
			SDL_PrivateKeyboard(SDL_RELEASED,
				SDL_Atari_TranslateKey(i, &keysym, SDL_FALSE));
			SDL_AtariIkbd_keyboard[i]=KEY_UNDEFINED;
		}
	}

	/*--- Send mouse events ---*/

	/* Mouse motion ? */
	if (SDL_AtariIkbd_mousex || SDL_AtariIkbd_mousey) {
		SDL_PrivateMouseMotion(0, 1, SDL_AtariIkbd_mousex, SDL_AtariIkbd_mousey);
		SDL_AtariIkbd_mousex = SDL_AtariIkbd_mousey = 0;
	}

	/* Mouse button ? */
	if (SDL_AtariIkbd_mouseb != atari_prevmouseb) {
		for (i=0;i<2;i++) {
			int curbutton, prevbutton;

			curbutton = SDL_AtariIkbd_mouseb & (1<<i);
			prevbutton = atari_prevmouseb & (1<<i);

			if (curbutton && !prevbutton) {
				SDL_PrivateMouseButton(SDL_PRESSED, atari_GetButton(i), 0, 0);
			}
			if (!curbutton && prevbutton) {
				SDL_PrivateMouseButton(SDL_RELEASED, atari_GetButton(i), 0, 0);
			}
		}
		atari_prevmouseb = SDL_AtariIkbd_mouseb;
	}
}

void AtariIkbd_ShutdownEvents(void)
{
	Supexec(SDL_AtariIkbdUninstall);
}
