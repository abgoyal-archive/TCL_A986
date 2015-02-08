
#include "SDL_config.h"

#ifdef SDL_TIMER_MINT


#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <mint/cookie.h>
#include <mint/sysvars.h>
#include <mint/osbind.h>
#include <mint/mintbind.h>

#include "SDL_timer.h"
#include "../SDL_timer_c.h"
#include "SDL_thread.h"

#include "SDL_vbltimer_s.h"

/* from audio/mint */
void SDL_MintAudio_CheckFpu(void);

/* The first ticks value of the application */
static Uint32 start;
static SDL_bool read_hz200_from_vbl = SDL_FALSE;
static int mint_present; /* can we use Syield() ? */

void SDL_StartTicks(void)
{
	void *old_stack;
	unsigned long dummy;

	/* Set first ticks value */
	old_stack = (void *)Super(0);
	start = *((volatile long *)_hz_200);
	Super(old_stack);

	start *= 5;	/* One _hz_200 tic is 5ms */

	mint_present = (Getcookie(C_MiNT, &dummy) == C_FOUND);
}

Uint32 SDL_GetTicks (void)
{
	Uint32 now = start;

	if (read_hz200_from_vbl) {
		now = SDL_Atari_hz200;
	} else {
		void *old_stack = (void *)Super(0);
		now = *((volatile long *)_hz_200);
		Super(old_stack);
	}

	return((now*5)-start);
}

void SDL_Delay (Uint32 ms)
{
	Uint32 now;

	now = SDL_GetTicks();
	while ((SDL_GetTicks()-now)<ms){
		if (mint_present) {
			Syield();
		}
	}
}

/* Data to handle a single periodic alarm */
static SDL_bool timer_installed=SDL_FALSE;

/* This is only called if the event thread is not running */
int SDL_SYS_TimerInit(void)
{
	void *old_stack;

	SDL_MintAudio_CheckFpu();

	/* Install RunTimer in vbl vector */
	old_stack = (void *)Super(0);
	timer_installed = !SDL_AtariVblInstall(SDL_ThreadedTimerCheck);
	Super(old_stack);

	if (!timer_installed) {
		return(-1);
	}

	read_hz200_from_vbl = SDL_TRUE;
	return(SDL_SetTimerThreaded(0));
}

void SDL_SYS_TimerQuit(void)
{
	/* Uninstall RunTimer vbl vector */
	if (timer_installed) {
		void *old_stack = (void *)Super(0);
		SDL_AtariVblUninstall(SDL_ThreadedTimerCheck);
		Super(old_stack);
		timer_installed = SDL_FALSE;
	}
	read_hz200_from_vbl = SDL_FALSE;
}

int SDL_SYS_StartTimer(void)
{
	SDL_SetError("Internal logic error: MiNT uses vbl timer");
	return(-1);
}

void SDL_SYS_StopTimer(void)
{
	return;
}

#endif /* SDL_TIMER_MINT */
