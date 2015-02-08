
#include "SDL_config.h"

#include <signal.h>
#include <be/kernel/OS.h>

typedef thread_id SYS_ThreadHandle;

/* Functions needed to work with system threads in other portions of SDL */
extern void SDL_MaskSignals(sigset_t *omask);
extern void SDL_UnmaskSignals(sigset_t *omask);
