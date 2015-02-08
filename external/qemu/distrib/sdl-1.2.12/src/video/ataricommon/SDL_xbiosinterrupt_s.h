
#include "SDL_config.h"


#ifndef _SDL_XBIOSINTERRUPT_S_H_
#define _SDL_XBIOSINTERRUPT_S_H_

#include <mint/osbind.h>

#include "SDL_stdinc.h"

/* Variables */

extern volatile Uint16 SDL_AtariXbios_mouselock;	/* mouse lock position */
extern volatile Uint16 SDL_AtariXbios_mouseb;	/* buttons */
extern volatile Sint16 SDL_AtariXbios_mousex;	/* X relative motion */
extern volatile Sint16 SDL_AtariXbios_mousey;	/* Y relative motion */
extern volatile Uint16 SDL_AtariXbios_joystick;	/* Joystick */

/* Functions */ 

extern void SDL_AtariXbios_Install(_KBDVECS *kbdvecs,void *newmousevector,void *newjoystickvector);
extern void SDL_AtariXbios_Restore(_KBDVECS *kbdvecs);
extern void SDL_AtariXbios_MouseVector(void *buf);
extern void SDL_AtariXbios_JoystickVector(void *buf);

#endif /* _SDL_XBIOSINTERRUPT_S_H_ */
