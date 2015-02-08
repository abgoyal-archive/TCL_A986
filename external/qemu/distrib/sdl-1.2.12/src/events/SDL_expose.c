
#include "SDL_config.h"

/* Refresh event handling code for SDL */

#include "SDL_events.h"
#include "SDL_events_c.h"


/* This is global for SDL_eventloop.c */
int SDL_PrivateExpose(void)
{
	int posted;
	SDL_Event events[32];

	/* Pull out all old refresh events */
	SDL_PeepEvents(events, sizeof(events)/sizeof(events[0]),
	                    SDL_GETEVENT, SDL_VIDEOEXPOSEMASK);

	/* Post the event, if desired */
	posted = 0;
	if ( SDL_ProcessEvents[SDL_VIDEOEXPOSE] == SDL_ENABLE ) {
		SDL_Event event;
		event.type = SDL_VIDEOEXPOSE;
		if ( (SDL_EventOK == NULL) || (*SDL_EventOK)(&event) ) {
			posted = 1;
			SDL_PushEvent(&event);
		}
	}
	return(posted);
}
