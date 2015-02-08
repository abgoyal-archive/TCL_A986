
#include "SDL_config.h"

#ifndef _SDL_DirectFB_video_h
#define _SDL_DirectFB_video_h

#include <directfb.h>

#include "SDL_mouse.h"
#include "../SDL_sysvideo.h"

#define _THIS SDL_VideoDevice *this

/* Private display data */

struct SDL_PrivateVideoData
{
  int                    initialized;

  IDirectFB             *dfb;
  IDirectFBDisplayLayer *layer;
  IDirectFBEventBuffer  *eventbuffer;

  int nummodes;
  SDL_Rect **modelist;

  /* MGA CRTC2 support */
  int enable_mga_crtc2;
  int mga_crtc2_stretch;
  float mga_crtc2_stretch_overscan;
  IDirectFBDisplayLayer *c2layer;
  IDirectFBSurface *c2frame;
  DFBRectangle c2ssize;	/* Real screen size */
  DFBRectangle c2dsize;	/* Stretched screen size */
  DFBRectangle c2framesize;    /* CRTC2 screen size */
};

#define HIDDEN (this->hidden)

void SetDirectFBerror (const char *function, DFBResult code);

#endif /* _SDL_DirectFB_video_h */
