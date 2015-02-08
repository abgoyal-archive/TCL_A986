
#ifndef __ASM_ARM_MACH_IRQ_H
#define __ASM_ARM_MACH_IRQ_H

#include <linux/irq.h>

struct seq_file;

extern void (*init_arch_irq)(void);
extern void init_FIQ(void);
extern int show_fiq_list(struct seq_file *, void *);

#define do_bad_IRQ(irq,desc)				\
do {							\
	raw_spin_lock(&desc->lock);			\
	handle_bad_irq(irq, desc);			\
	raw_spin_unlock(&desc->lock);			\
} while(0)

#endif
