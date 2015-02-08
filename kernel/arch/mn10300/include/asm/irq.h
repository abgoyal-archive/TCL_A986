
#ifndef _ASM_IRQ_H
#define _ASM_IRQ_H

#include <asm/intctl-regs.h>
#include <asm/reset-regs.h>
#include <proc/irq.h>

/* this number is used when no interrupt has been assigned */
#define NO_IRQ		INT_MAX

/* hardware irq numbers */
#define NR_IRQS		GxICR_NUM_IRQS

/* external hardware irq numbers */
#define NR_XIRQS	GxICR_NUM_XIRQS

#define irq_canonicalize(IRQ) (IRQ)

#endif /* _ASM_IRQ_H */
