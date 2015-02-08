
#include "SDL_config.h"

#include "../video/SDL_sysvideo.h"

/* Useful functions and variables from SDL_sysevents.c */

#ifdef __BEOS__		/* The Be event loop runs in a separate thread */
#define MUST_THREAD_EVENTS
#endif

#ifdef __WIN32__	/* Win32 doesn't allow a separate event thread */
#define CANT_THREAD_EVENTS
#endif

#ifdef IPOD			/* iPod doesn't support threading at all */
#define CANT_THREAD_EVENTS
#endif

#ifdef __MACOS__	/* MacOS 7/8 don't support preemptive multi-tasking */
#define CANT_THREAD_EVENTS
#endif

#ifdef __OS2__		/* The OS/2 event loop runs in a separate thread */
#define MUST_THREAD_EVENTS
#endif
