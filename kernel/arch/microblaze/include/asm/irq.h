

#ifndef _ASM_MICROBLAZE_IRQ_H
#define _ASM_MICROBLAZE_IRQ_H

#define NR_IRQS 32
#include <asm-generic/irq.h>

#include <linux/interrupt.h>

typedef unsigned long irq_hw_number_t;

extern unsigned int nr_irq;

#define NO_IRQ (-1)

struct pt_regs;
extern void do_IRQ(struct pt_regs *regs);

struct device_node;
extern unsigned int irq_of_parse_and_map(struct device_node *dev, int index);

static inline void irq_dispose_mapping(unsigned int virq)
{
	return;
}

struct irq_host;

extern unsigned int irq_create_mapping(struct irq_host *host,
					irq_hw_number_t hwirq);

extern unsigned int irq_create_of_mapping(struct device_node *controller,
					u32 *intspec, unsigned int intsize);

#endif /* _ASM_MICROBLAZE_IRQ_H */
