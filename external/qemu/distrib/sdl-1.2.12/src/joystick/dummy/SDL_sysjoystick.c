
#include "SDL_config.h"

#if defined(SDL_JOYSTICK_DUMMY) || defined(SDL_JOYSTICK_DISABLED)

/* This is the system specific header for the SDL joystick API */

#include "SDL_joystick.h"
#include "../SDL_sysjoystick.h"
#include "../SDL_joystick_c.h"

int SDL_SYS_JoystickInit(void)
{
	SDL_numjoysticks = 0;
	return(0);
}

/* Function to get the device-dependent name of a joystick */
const char *SDL_SYS_JoystickName(int index)
{
	SDL_SetError("Logic error: No joysticks available");
	return(NULL);
}

int SDL_SYS_JoystickOpen(SDL_Joystick *joystick)
{
	SDL_SetError("Logic error: No joysticks available");
	return(-1);
}

void SDL_SYS_JoystickUpdate(SDL_Joystick *joystick)
{
	return;
}

/* Function to close a joystick after use */
void SDL_SYS_JoystickClose(SDL_Joystick *joystick)
{
	return;
}

/* Function to perform any system-specific joystick related cleanup */
void SDL_SYS_JoystickQuit(void)
{
	return;
}

#endif /* SDL_JOYSTICK_DUMMY || SDL_JOYSTICK_DISABLED */
