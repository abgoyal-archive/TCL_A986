
#include "SDL_config.h"

/* Define this if you want surface leak detection code enabled */
/*#define CHECK_LEAKS*/

/* Global variables used to check leaks in code using SDL */

#ifdef CHECK_LEAKS
extern int surfaces_allocated;
#endif
