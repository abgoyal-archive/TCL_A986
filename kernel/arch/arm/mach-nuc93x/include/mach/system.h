

#include <asm/proc-fns.h>

static void arch_idle(void)
{
}

static void arch_reset(char mode, const char *cmd)
{
	cpu_reset(0);
}

