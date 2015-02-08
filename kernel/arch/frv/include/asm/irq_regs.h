

#ifndef _ASM_IRQ_REGS_H
#define _ASM_IRQ_REGS_H

#define ARCH_HAS_OWN_IRQ_REGS

#ifndef __ASSEMBLY__
#define get_irq_regs() (__frame)
#endif

#endif /* _ASM_IRQ_REGS_H */
