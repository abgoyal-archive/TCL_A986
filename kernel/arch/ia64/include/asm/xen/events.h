
#ifndef _ASM_IA64_XEN_EVENTS_H
#define _ASM_IA64_XEN_EVENTS_H

enum ipi_vector {
	XEN_RESCHEDULE_VECTOR,
	XEN_IPI_VECTOR,
	XEN_CMCP_VECTOR,
	XEN_CPEP_VECTOR,

	XEN_NR_IPIS,
};

static inline int xen_irqs_disabled(struct pt_regs *regs)
{
	return !(ia64_psr(regs)->i);
}

#define irq_ctx_init(cpu)	do { } while (0)

#endif /* _ASM_IA64_XEN_EVENTS_H */
