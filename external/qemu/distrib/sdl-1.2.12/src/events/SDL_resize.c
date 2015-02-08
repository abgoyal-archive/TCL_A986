
#include "SDL_config.h"

/* Resize event handling code for SDL */

#include "SDL_events.h"
#include "SDL_events_c.h"
#include "../video/SDL_sysvideo.h"


/* Keep the last resize event so we don't post duplicates */
static struct {
	int w;
	int h;
} last_resize;

/* This is global for SDL_eventloop.c */
int SDL_PrivateResize(int w, int h)
{
	int posted;
	SDL_Event events[32];

	/* See if this event would change the video surface */
	if ( !w || !h
#ifndef __OS2__
            || ((last_resize.w == w) && (last_resize.h == h))
#endif
    ) {
		return(0);
	}
        last_resize.w = w;
        last_resize.h = h;
	if ( ! SDL_VideoSurface ||
	     ((w == SDL_VideoSurface->w) && (h == SDL_VideoSurface->h)) ) {
		return(0);
	}

	/* Pull out all old resize events */
	SDL_PeepEvents(events, sizeof(events)/sizeof(events[0]),
	                    SDL_GETEVENT, SDL_VIDEORESIZEMASK);

	/* Post the event, if desired */
	posted = 0;
	if ( SDL_ProcessEvents[SDL_VIDEORESIZE] == SDL_ENABLE ) {
		SDL_Event event;
		event.type = SDL_VIDEORESIZE;
		event.resize.w = w;
		event.resize.h = h;
		if ( (SDL_EventOK == NULL) || (*SDL_EventOK)(&event) ) {
			posted = 1;
			SDL_PushEvent(&event);
		}
	}
	return(posted);
}
