
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

static inline void arch_idle(void)
{
	const char *irq_stat = (char *)0xff000000;

	/* disable clock switching */
	asm volatile ("mcr p15, 0, ip, c15, c2, 2" : : : "cc");

	/* wait for an interrupt to occur */
	while (!*irq_stat);

	/* enable clock switching */
	asm volatile ("mcr p15, 0, ip, c15, c1, 2" : : : "cc");
}

#define arch_reset(mode, cmd)	cpu_reset(0x80000000)

#endif
