
#ifndef __ASM_ARM_MACH_TIME_H
#define __ASM_ARM_MACH_TIME_H

#include <linux/sysdev.h>

struct sys_timer {
	struct sys_device	dev;
	void			(*init)(void);
	void			(*suspend)(void);
	void			(*resume)(void);
#ifdef CONFIG_ARCH_USES_GETTIMEOFFSET
	unsigned long		(*offset)(void);
#endif
};

extern struct sys_timer *system_timer;
extern void timer_tick(void);

#endif
