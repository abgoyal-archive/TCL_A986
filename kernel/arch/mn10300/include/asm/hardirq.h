
#ifndef _ASM_HARDIRQ_H
#define _ASM_HARDIRQ_H

#include <linux/threads.h>
#include <linux/irq.h>
#include <asm/exceptions.h>

/* assembly code in softirq.h is sensitive to the offsets of these fields */
typedef struct {
	unsigned int	__softirq_pending;
	unsigned long	idle_timestamp;
	unsigned int	__nmi_count;	/* arch dependent */
	unsigned int	__irq_count;	/* arch dependent */
} ____cacheline_aligned irq_cpustat_t;

#include <linux/irq_cpustat.h>	/* Standard mappings for irq_cpustat_t above */

extern void ack_bad_irq(int irq);

typedef void (*intr_stub_fnx)(struct pt_regs *regs,
			      enum exception_code intcode);

extern asmlinkage void set_excp_vector(enum exception_code code,
				       intr_stub_fnx handler);

#endif /* _ASM_HARDIRQ_H */
