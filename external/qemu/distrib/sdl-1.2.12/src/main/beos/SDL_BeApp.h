
#include "SDL_config.h"

/* Handle the BeApp specific portions of the application */

/* Initialize the Be Application, if it's not already started */
extern int SDL_InitBeApp(void);

/* Quit the Be Application, if there's nothing left to do */
extern void SDL_QuitBeApp(void);

/* Flag to tell whether the app is active or not */
extern int SDL_BeAppActive;
