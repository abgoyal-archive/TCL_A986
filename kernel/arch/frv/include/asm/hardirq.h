

#ifndef __ASM_HARDIRQ_H
#define __ASM_HARDIRQ_H

#include <asm/atomic.h>

extern atomic_t irq_err_count;
static inline void ack_bad_irq(int irq)
{
	atomic_inc(&irq_err_count);
}
#define ack_bad_irq ack_bad_irq

#include <asm-generic/hardirq.h>

#endif
