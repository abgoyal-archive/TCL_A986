

#ifndef _ASM_MICROBLAZE_HARDIRQ_H
#define _ASM_MICROBLAZE_HARDIRQ_H

/* should be defined in each interrupt controller driver */
extern unsigned int get_irq(struct pt_regs *regs);

#include <asm-generic/hardirq.h>

#endif /* _ASM_MICROBLAZE_HARDIRQ_H */
