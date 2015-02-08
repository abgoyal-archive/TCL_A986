

#include <asm/proc-fns.h>
#include "hardware.h"
#include "pxa2xx-regs.h"

static inline void arch_idle(void)
{
	cpu_do_idle();
}


void arch_reset(char mode, const char *cmd);
