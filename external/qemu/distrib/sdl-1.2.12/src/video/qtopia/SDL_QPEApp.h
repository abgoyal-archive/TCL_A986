
#include "SDL_config.h"

/* Handle the QPE application loop */

/* Initialize the QPE Application, if it's not already started */
extern int SDL_InitQPEApp(void);

/* Quit the QPE Application, if there's nothing left to do */
extern void SDL_QuitQPEApp(void);

/* Flag to tell whether the app is active or not */
extern int SDL_QPEAppActive;
