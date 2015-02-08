
#include "SDL_config.h"

#ifndef _SDL_dspvideo_h
#define _SDL_dspvideo_h

#if TARGET_API_MAC_OSX
#  include <DrawSprocket/DrawSprocket.h> /* Drawsprocket.framework */
#else
#include <DrawSprocket.h>
#endif

#include "../maccommon/SDL_lowvideo.h"

/* DrawSprocket specific information */
struct DSpInfo {
	DSpContextReference dsp_context;
	CGrafPtr            dsp_back_buffer;
   int                 dsp_old_depth;
   
	/* Flags for hw acceleration */
	int dsp_vram_available;
	int dsp_agp_available;
	
	
}; 
/* Old variable names */
#define dsp_context (this->hidden->dspinfo->dsp_context)
#define dsp_back_buffer (this->hidden->dspinfo->dsp_back_buffer)
#define dsp_old_depth   (this->hidden->dspinfo->dsp_old_depth)
#define dsp_vram_available (this->hidden->dspinfo->dsp_vram_available)
#define dsp_agp_available (this->hidden->dspinfo->dsp_agp_available)

#endif /* _SDL_dspvideo_h */
