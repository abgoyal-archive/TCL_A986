
#ifndef __M68K_HARDIRQ_H
#define __M68K_HARDIRQ_H

#include <asm/irq.h>

#define HARDIRQ_BITS	8

#if (1 << HARDIRQ_BITS) < NR_IRQS
# error HARDIRQ_BITS is too low!
#endif

#include <asm-generic/hardirq.h>

#endif /* __M68K_HARDIRQ_H */
