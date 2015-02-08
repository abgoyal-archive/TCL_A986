
#include "SDL_config.h"


#ifndef _SDL_IKBDINTERRUPT_S_H_
#define _SDL_IKBDINTERRUPT_S_H_

#include <mint/osbind.h>

#include "SDL_stdinc.h"

/* Const */

#define IKBD_JOY_UP		(1<<0)
#define IKBD_JOY_DOWN	(1<<1)
#define IKBD_JOY_LEFT	(1<<2)
#define IKBD_JOY_RIGHT	(1<<3)
#define IKBD_JOY_FIRE	(1<<7)

/* Variables */

extern volatile Uint8  SDL_AtariIkbd_keyboard[128];	/* Keyboard table */
extern volatile Uint16 SDL_AtariIkbd_mouseb;	/* Mouse on port 0, buttons */
extern volatile Sint16 SDL_AtariIkbd_mousex;	/* Mouse X relative motion */
extern volatile Sint16 SDL_AtariIkbd_mousey;	/* Mouse Y relative motion */
extern volatile Uint16 SDL_AtariIkbd_joystick;	/* Joystick on port 1 */

/* For joystick driver to know if this is usable */
extern Uint16 SDL_AtariIkbd_enabled;
										
/* Functions */ 

extern void SDL_AtariIkbdInstall(void);
extern void SDL_AtariIkbdUninstall(void);

#endif /* _SDL_IKBDINTERRUPT_S_H_ */
