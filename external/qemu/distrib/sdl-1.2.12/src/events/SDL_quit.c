
#include "SDL_config.h"

/* General quit handling code for SDL */

#ifdef HAVE_SIGNAL_H
#include <signal.h>
#endif

#include "SDL_events.h"
#include "SDL_events_c.h"


#ifdef HAVE_SIGNAL_H
static void SDL_HandleSIG(int sig)
{
	/* Reset the signal handler */
	signal(sig, SDL_HandleSIG);

	/* Signal a quit interrupt */
	SDL_PrivateQuit();
}
#endif /* HAVE_SIGNAL_H */

/* Public functions */
int SDL_QuitInit(void)
{
#ifdef HAVE_SIGNAL_H
	void (*ohandler)(int);

	/* Both SIGINT and SIGTERM are translated into quit interrupts */
	ohandler = signal(SIGINT, SDL_HandleSIG);
	if ( ohandler != SIG_DFL )
		signal(SIGINT, ohandler);
	ohandler = signal(SIGTERM, SDL_HandleSIG);
	if ( ohandler != SIG_DFL )
		signal(SIGTERM, ohandler);
#endif /* HAVE_SIGNAL_H */

	/* That's it! */
	return(0);
}
void SDL_QuitQuit(void)
{
#ifdef HAVE_SIGNAL_H
	void (*ohandler)(int);

	ohandler = signal(SIGINT, SIG_DFL);
	if ( ohandler != SDL_HandleSIG )
		signal(SIGINT, ohandler);
	ohandler = signal(SIGTERM, SIG_DFL);
	if ( ohandler != SDL_HandleSIG )
		signal(SIGTERM, ohandler);
#endif /* HAVE_SIGNAL_H */
}

/* This function returns 1 if it's okay to close the application window */
int SDL_PrivateQuit(void)
{
	int posted;

	posted = 0;
	if ( SDL_ProcessEvents[SDL_QUIT] == SDL_ENABLE ) {
		SDL_Event event;
		event.type = SDL_QUIT;
		if ( (SDL_EventOK == NULL) || (*SDL_EventOK)(&event) ) {
			posted = 1;
			SDL_PushEvent(&event);
		}
	}
	return(posted);
}
