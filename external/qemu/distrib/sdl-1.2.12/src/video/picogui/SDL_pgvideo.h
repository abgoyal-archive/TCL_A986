
#include "SDL_config.h"

#ifndef _SDL_pgvideo_h
#define _SDL_pgvideo_h

#include "SDL_mouse.h"
#include "SDL_mutex.h"
#include "../SDL_sysvideo.h"

#include <picogui.h>
#include <sys/shm.h>

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this


/* Private display data */

struct SDL_PrivateVideoData {
  pghandle wApp, wCanvas;        /* PicoGUI widgets */
  pghandle bitmap;
  struct pgshmbitmap shm;        /* shared memory info */
  struct pgmodeinfo mi;          /* PicoGUI video mode info structure */
};

#endif /* _SDL_pgvideo_h */
