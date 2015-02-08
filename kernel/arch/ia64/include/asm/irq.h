
#ifndef _ASM_IA64_IRQ_H
#define _ASM_IA64_IRQ_H


#include <linux/types.h>
#include <linux/cpumask.h>
#include <generated/nr-irqs.h>

static __inline__ int
irq_canonicalize (int irq)
{
	/*
	 * We do the legacy thing here of pretending that irqs < 16
	 * are 8259 irqs.  This really shouldn't be necessary at all,
	 * but we keep it here as serial.c still uses it...
	 */
	return ((irq == 2) ? 9 : irq);
}

extern void set_irq_affinity_info (unsigned int irq, int dest, int redir);
bool is_affinity_mask_valid(const struct cpumask *cpumask);

#define is_affinity_mask_valid is_affinity_mask_valid

#endif /* _ASM_IA64_IRQ_H */
