

#include <signal.h>

extern void handle_signal(int sig, struct sigcontext *sc);

void hard_handler(int sig)
{
	handle_signal(sig, (struct sigcontext *) (&sig + 1));
}
