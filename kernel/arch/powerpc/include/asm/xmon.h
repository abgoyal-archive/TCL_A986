
#ifndef __ASM_POWERPC_XMON_H
#define __ASM_POWERPC_XMON_H


#ifdef __KERNEL__

#include <linux/irqreturn.h>

#ifdef CONFIG_XMON
extern void xmon_setup(void);
extern void xmon_register_spus(struct list_head *list);
struct pt_regs;
extern int xmon(struct pt_regs *excp);
extern irqreturn_t xmon_irq(int, void *);
#else
static inline void xmon_setup(void) { };
static inline void xmon_register_spus(struct list_head *list) { };
#endif

#if defined(CONFIG_XMON) && defined(CONFIG_SMP)
extern int cpus_are_in_xmon(void);
#endif

#endif /* __KERNEL __ */
#endif /* __ASM_POWERPC_XMON_H */
