
#include <linux/module.h>
#include <asm/irq_regs.h>

#ifndef ARCH_HAS_OWN_IRQ_REGS
DEFINE_PER_CPU(struct pt_regs *, __irq_regs);
EXPORT_PER_CPU_SYMBOL(__irq_regs);
#endif
