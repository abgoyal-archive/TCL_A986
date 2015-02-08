

#ifndef __ASM_ARCH_PNX4008_PM_H
#define __ASM_ARCH_PNX4008_PM_H

#ifndef __ASSEMBLER__
#include "irq.h"
#include "irqs.h"
#include "clock.h"

extern void pnx4008_pm_idle(void);
extern void pnx4008_pm_suspend(void);
extern unsigned int pnx4008_cpu_suspend_sz;
extern void pnx4008_cpu_suspend(void);
extern unsigned int pnx4008_cpu_standby_sz;
extern void pnx4008_cpu_standby(void);

extern int pnx4008_startup_pll(struct clk *);
extern int pnx4008_shutdown_pll(struct clk *);

#endif				/* ASSEMBLER */
#endif				/* __ASM_ARCH_PNX4008_PM_H */
