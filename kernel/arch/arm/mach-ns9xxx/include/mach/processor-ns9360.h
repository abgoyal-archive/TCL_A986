
#ifndef __ASM_ARCH_PROCESSORNS9360_H
#define __ASM_ARCH_PROCESSORNS9360_H

#include <linux/init.h>

void ns9360_reset(char mode);

unsigned long ns9360_systemclock(void) __attribute__((const));

static inline unsigned long ns9360_cpuclock(void) __attribute__((const));
static inline unsigned long ns9360_cpuclock(void)
{
	return ns9360_systemclock() / 2;
}

void __init ns9360_map_io(void);

extern struct sys_timer ns9360_timer;

int ns9360_gpio_configure(unsigned gpio, int inv, int func);

#endif /* ifndef __ASM_ARCH_PROCESSORNS9360_H */
