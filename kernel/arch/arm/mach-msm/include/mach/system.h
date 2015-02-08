

#include <mach/hardware.h>

void arch_idle(void);

static inline void arch_reset(char mode, const char *cmd)
{
	for (;;) ;  /* depends on IPC w/ other core */
}

extern void (*msm_hw_reset_hook)(void);
