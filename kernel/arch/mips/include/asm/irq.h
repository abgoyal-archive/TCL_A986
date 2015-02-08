
#ifndef _ASM_IRQ_H
#define _ASM_IRQ_H

#include <linux/linkage.h>
#include <linux/smp.h>

#include <asm/mipsmtregs.h>

#include <irq.h>

#ifdef CONFIG_I8259
static inline int irq_canonicalize(int irq)
{
	return ((irq == I8259A_IRQ_BASE + 2) ? I8259A_IRQ_BASE + 9 : irq);
}
#else
#define irq_canonicalize(irq) (irq)	/* Sane hardware, sane code ... */
#endif

#ifdef CONFIG_MIPS_MT_SMTC

struct irqaction;

extern unsigned long irq_hwmask[];
extern int setup_irq_smtc(unsigned int irq, struct irqaction * new,
                          unsigned long hwmask);

static inline void smtc_im_ack_irq(unsigned int irq)
{
	if (irq_hwmask[irq] & ST0_IM)
		set_c0_status(irq_hwmask[irq] & ST0_IM);
}

#else

static inline void smtc_im_ack_irq(unsigned int irq)
{
}

#endif /* CONFIG_MIPS_MT_SMTC */

#ifdef CONFIG_MIPS_MT_SMTC_IRQAFF
#include <linux/cpumask.h>

extern int plat_set_irq_affinity(unsigned int irq,
				  const struct cpumask *affinity);
extern void smtc_forward_irq(unsigned int irq);

#define IRQ_AFFINITY_HOOK(irq)						\
do {									\
    if (!cpumask_test_cpu(smp_processor_id(), irq_desc[irq].affinity)) {\
	smtc_forward_irq(irq);						\
	irq_exit();							\
	return;								\
    }									\
} while (0)

#else /* Not doing SMTC affinity */

#define IRQ_AFFINITY_HOOK(irq) do { } while (0)

#endif /* CONFIG_MIPS_MT_SMTC_IRQAFF */

#ifdef CONFIG_MIPS_MT_SMTC_IM_BACKSTOP

#define __DO_IRQ_SMTC_HOOK(irq)						\
do {									\
	IRQ_AFFINITY_HOOK(irq);						\
	if (irq_hwmask[irq] & 0x0000ff00)				\
		write_c0_tccontext(read_c0_tccontext() &		\
				   ~(irq_hwmask[irq] & 0x0000ff00));	\
} while (0)

#define __NO_AFFINITY_IRQ_SMTC_HOOK(irq)				\
do {									\
	if (irq_hwmask[irq] & 0x0000ff00)                               \
		write_c0_tccontext(read_c0_tccontext() &		\
				   ~(irq_hwmask[irq] & 0x0000ff00));	\
} while (0)

#else

#define __DO_IRQ_SMTC_HOOK(irq)						\
do {									\
	IRQ_AFFINITY_HOOK(irq);						\
} while (0)
#define __NO_AFFINITY_IRQ_SMTC_HOOK(irq) do { } while (0)

#endif

extern void do_IRQ(unsigned int irq);

#ifdef CONFIG_MIPS_MT_SMTC_IRQAFF

extern void do_IRQ_no_affinity(unsigned int irq);

#endif /* CONFIG_MIPS_MT_SMTC_IRQAFF */

extern void arch_init_irq(void);
extern void spurious_interrupt(void);

extern int allocate_irqno(void);
extern void alloc_legacy_irqno(void);
extern void free_irqno(unsigned int irq);

#define CP0_LEGACY_COMPARE_IRQ 7

extern int cp0_compare_irq;
extern int cp0_compare_irq_shift;
extern int cp0_perfcount_irq;

#endif /* _ASM_IRQ_H */
