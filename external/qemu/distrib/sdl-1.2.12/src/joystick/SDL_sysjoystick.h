
#include "SDL_config.h"

/* This is the system specific header for the SDL joystick API */

#include "SDL_joystick.h"

/* The SDL joystick structure */
struct _SDL_Joystick {
	Uint8 index;		/* Device index */
	const char *name;	/* Joystick name - system dependent */

	int naxes;		/* Number of axis controls on the joystick */
	Sint16 *axes;		/* Current axis states */

	int nhats;		/* Number of hats on the joystick */
	Uint8 *hats;		/* Current hat states */
	
	int nballs;		/* Number of trackballs on the joystick */
	struct balldelta {
		int dx;
		int dy;
	} *balls;		/* Current ball motion deltas */
	
	int nbuttons;		/* Number of buttons on the joystick */
	Uint8 *buttons;		/* Current button states */
	
	struct joystick_hwdata *hwdata;	/* Driver dependent information */

	int ref_count;		/* Reference count for multiple opens */
};

extern int SDL_SYS_JoystickInit(void);

/* Function to get the device-dependent name of a joystick */
extern const char *SDL_SYS_JoystickName(int index);

extern int SDL_SYS_JoystickOpen(SDL_Joystick *joystick);

extern void SDL_SYS_JoystickUpdate(SDL_Joystick *joystick);

/* Function to close a joystick after use */
extern void SDL_SYS_JoystickClose(SDL_Joystick *joystick);

/* Function to perform any system-specific joystick related cleanup */
extern void SDL_SYS_JoystickQuit(void);

