

#ifndef _ASM_IRQ_H_
#define _ASM_IRQ_H_

#define NR_IRQS				48
#define IRQ_BASE_CPU			(0 * 16)
#define IRQ_BASE_FPGA			(1 * 16)
#define IRQ_BASE_MB93493		(2 * 16)

/* probe returns a 32-bit IRQ mask:-/ */
#define MIN_PROBE_IRQ			(NR_IRQS - 32)

#ifndef __ASSEMBLY__
static inline int irq_canonicalize(int irq)
{
	return irq;
}
#endif

#endif /* _ASM_IRQ_H_ */
