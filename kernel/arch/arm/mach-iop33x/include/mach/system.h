
#include <asm/hardware/iop3xx.h>

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	*IOP3XX_PCSR = 0x30;

	/* Jump into ROM at address 0 */
	cpu_reset(0);
}
