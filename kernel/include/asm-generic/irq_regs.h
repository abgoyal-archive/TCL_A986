

#ifndef _ASM_GENERIC_IRQ_REGS_H
#define _ASM_GENERIC_IRQ_REGS_H

#include <linux/percpu.h>

DECLARE_PER_CPU(struct pt_regs *, __irq_regs);

static inline struct pt_regs *get_irq_regs(void)
{
	return __get_cpu_var(__irq_regs);
}

static inline struct pt_regs *set_irq_regs(struct pt_regs *new_regs)
{
	struct pt_regs *old_regs, **pp_regs = &__get_cpu_var(__irq_regs);

	old_regs = *pp_regs;
	*pp_regs = new_regs;
	return old_regs;
}

#endif /* _ASM_GENERIC_IRQ_REGS_H */
