

#ifndef _SPARC_IRQ_H
#define _SPARC_IRQ_H

#define NR_IRQS    16

#include <linux/interrupt.h>

#define irq_canonicalize(irq)	(irq)

extern void __init init_IRQ(void);
#endif
