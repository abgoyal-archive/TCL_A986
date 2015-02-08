
#include "SDL_config.h"

#ifndef _SDL_wsconsvideo_h
#define _SDL_wsconsvideo_h

#include <sys/time.h>
#include <termios.h>
#include <dev/wscons/wsconsio.h>

#include "SDL_mouse.h"
#include "SDL_mutex.h"
#include "../SDL_sysvideo.h"

void WSCONS_ReportError(char *fmt, ...);

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *this
#define private	(this->hidden)

/* Private display data */

typedef void WSCONS_bitBlit(Uint8 *src_pos,
			    int srcRightDelta, // pixels, not bytes
			    int srcDownDelta,  // pixels, not bytes
			    Uint8 *dst_pos,
			    int dst_linebytes,
			    int width,
			    int height);

struct SDL_PrivateVideoData {
  int fd;                       /* file descriptor of open device */
  struct wsdisplay_fbinfo info; /* frame buffer characteristics */
  int physlinebytes;            /* number of bytes per row */
  int redMask, greenMask, blueMask;

  Uint8 *fbstart;               /* These refer to the surface used, */
  int fblinebytes;              /* physical frame buffer or shadow. */

  size_t fbmem_len;
  Uint8 *physmem;
  Uint8 *shadowmem;
  int rotate;
  int shadowFB;                 /* Tells whether a shadow is being used. */

  WSCONS_bitBlit *blitFunc;

  SDL_Rect *SDL_modelist[2];

  unsigned int kbdType;
  int did_save_tty;
  struct termios saved_tty;
};


#endif /* _SDL_wsconsvideo_h */
