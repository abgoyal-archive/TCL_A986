
#include "SDL_config.h"

/* Useful functions and variables from SDL_joystick.c */
#include "SDL_joystick.h"

/* The number of available joysticks on the system */
extern Uint8 SDL_numjoysticks;

/* Internal event queueing functions */
extern int SDL_PrivateJoystickAxis(SDL_Joystick *joystick,
                                   Uint8 axis, Sint16 value);
extern int SDL_PrivateJoystickBall(SDL_Joystick *joystick,
                                   Uint8 ball, Sint16 xrel, Sint16 yrel);
extern int SDL_PrivateJoystickHat(SDL_Joystick *joystick,
                                 Uint8 hat, Uint8 value);
extern int SDL_PrivateJoystickButton(SDL_Joystick *joystick,
                                     Uint8 button, Uint8 state);
