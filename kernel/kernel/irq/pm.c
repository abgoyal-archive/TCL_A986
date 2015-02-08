

#include <linux/irq.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include "internals.h"

void suspend_device_irqs(void)
{
	struct irq_desc *desc;
	int irq;

	for_each_irq_desc(irq, desc) {
		unsigned long flags;

		raw_spin_lock_irqsave(&desc->lock, flags);
		__disable_irq(desc, irq, true);
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}

	for_each_irq_desc(irq, desc)
		if (desc->status & IRQ_SUSPENDED)
			synchronize_irq(irq);
}
EXPORT_SYMBOL_GPL(suspend_device_irqs);

void resume_device_irqs(void)
{
	struct irq_desc *desc;
	int irq;

	for_each_irq_desc(irq, desc) {
		unsigned long flags;

		if (!(desc->status & IRQ_SUSPENDED))
			continue;

		raw_spin_lock_irqsave(&desc->lock, flags);
		__enable_irq(desc, irq, true);
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
}
EXPORT_SYMBOL_GPL(resume_device_irqs);

int check_wakeup_irqs(void)
{
	struct irq_desc *desc;
	int irq;

	for_each_irq_desc(irq, desc)
		if ((desc->status & IRQ_WAKEUP) && (desc->status & IRQ_PENDING))
			return -EBUSY;

	return 0;
}
