
#include "SDL_config.h"

/* General fatal signal handling code for SDL */

#ifdef HAVE_SIGNAL_H

#include <signal.h>

#include "SDL.h"
#include "SDL_fatal.h"


static void SDL_Parachute(int sig)
{
	signal(sig, SIG_DFL);
	SDL_Quit();
	raise(sig);
}

static int SDL_fatal_signals[] = {
	SIGSEGV,
#ifdef SIGBUS
	SIGBUS,
#endif
#ifdef SIGFPE
	SIGFPE,
#endif
#ifdef SIGQUIT
	SIGQUIT,
#endif
	0
};

void SDL_InstallParachute(void)
{
	/* Set a handler for any fatal signal not already handled */
	int i;
#ifdef HAVE_SIGACTION
	struct sigaction action;

	for ( i=0; SDL_fatal_signals[i]; ++i ) {
		sigaction(SDL_fatal_signals[i], NULL, &action);
		if ( action.sa_handler == SIG_DFL ) {
			action.sa_handler = SDL_Parachute;
			sigaction(SDL_fatal_signals[i], &action, NULL);
		}
	}
#ifdef SIGALRM
	/* Set SIGALRM to be ignored -- necessary on Solaris */
	sigaction(SIGALRM, NULL, &action);
	if ( action.sa_handler == SIG_DFL ) {
		action.sa_handler = SIG_IGN;
		sigaction(SIGALRM, &action, NULL);
	}
#endif
#else
	void (*ohandler)(int);

	for ( i=0; SDL_fatal_signals[i]; ++i ) {
		ohandler = signal(SDL_fatal_signals[i], SDL_Parachute);
		if ( ohandler != SIG_DFL ) {
			signal(SDL_fatal_signals[i], ohandler);
		}
	}
#endif /* HAVE_SIGACTION */
	return;
}

void SDL_UninstallParachute(void)
{
	/* Remove a handler for any fatal signal handled */
	int i;
#ifdef HAVE_SIGACTION
	struct sigaction action;

	for ( i=0; SDL_fatal_signals[i]; ++i ) {
		sigaction(SDL_fatal_signals[i], NULL, &action);
		if ( action.sa_handler == SDL_Parachute ) {
			action.sa_handler = SIG_DFL;
			sigaction(SDL_fatal_signals[i], &action, NULL);
		}
	}
#else
	void (*ohandler)(int);

	for ( i=0; SDL_fatal_signals[i]; ++i ) {
		ohandler = signal(SDL_fatal_signals[i], SIG_DFL);
		if ( ohandler != SDL_Parachute ) {
			signal(SDL_fatal_signals[i], ohandler);
		}
	}
#endif /* HAVE_SIGACTION */
}

#else

/* No signals on this platform, nothing to do.. */

void SDL_InstallParachute(void)
{
	return;
}

void SDL_UninstallParachute(void)
{
	return;
}

#endif /* HAVE_SIGNAL_H */
