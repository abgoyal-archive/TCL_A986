

#include <linux/irq.h>
#include <linux/msi.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>

#include "internals.h"

static void dynamic_irq_init_x(unsigned int irq, bool keep_chip_data)
{
	struct irq_desc *desc;
	unsigned long flags;

	desc = irq_to_desc(irq);
	if (!desc) {
		WARN(1, KERN_ERR "Trying to initialize invalid IRQ%d\n", irq);
		return;
	}

	/* Ensure we don't have left over values from a previous use of this irq */
	raw_spin_lock_irqsave(&desc->lock, flags);
	desc->status = IRQ_DISABLED;
	desc->chip = &no_irq_chip;
	desc->handle_irq = handle_bad_irq;
	desc->depth = 1;
	desc->msi_desc = NULL;
	desc->handler_data = NULL;
	if (!keep_chip_data)
		desc->chip_data = NULL;
	desc->action = NULL;
	desc->irq_count = 0;
	desc->irqs_unhandled = 0;
#ifdef CONFIG_SMP
	cpumask_setall(desc->affinity);
#ifdef CONFIG_GENERIC_PENDING_IRQ
	cpumask_clear(desc->pending_mask);
#endif
#endif
	raw_spin_unlock_irqrestore(&desc->lock, flags);
}

void dynamic_irq_init(unsigned int irq)
{
	dynamic_irq_init_x(irq, false);
}

void dynamic_irq_init_keep_chip_data(unsigned int irq)
{
	dynamic_irq_init_x(irq, true);
}

static void dynamic_irq_cleanup_x(unsigned int irq, bool keep_chip_data)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc) {
		WARN(1, KERN_ERR "Trying to cleanup invalid IRQ%d\n", irq);
		return;
	}

	raw_spin_lock_irqsave(&desc->lock, flags);
	if (desc->action) {
		raw_spin_unlock_irqrestore(&desc->lock, flags);
		WARN(1, KERN_ERR "Destroying IRQ%d without calling free_irq\n",
			irq);
		return;
	}
	desc->msi_desc = NULL;
	desc->handler_data = NULL;
	if (!keep_chip_data)
		desc->chip_data = NULL;
	desc->handle_irq = handle_bad_irq;
	desc->chip = &no_irq_chip;
	desc->name = NULL;
	clear_kstat_irqs(desc);
	raw_spin_unlock_irqrestore(&desc->lock, flags);
}

void dynamic_irq_cleanup(unsigned int irq)
{
	dynamic_irq_cleanup_x(irq, false);
}

void dynamic_irq_cleanup_keep_chip_data(unsigned int irq)
{
	dynamic_irq_cleanup_x(irq, true);
}


int set_irq_chip(unsigned int irq, struct irq_chip *chip)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc) {
		WARN(1, KERN_ERR "Trying to install chip for IRQ%d\n", irq);
		return -EINVAL;
	}

	if (!chip)
		chip = &no_irq_chip;

	raw_spin_lock_irqsave(&desc->lock, flags);
	irq_chip_set_defaults(chip);
	desc->chip = chip;
	raw_spin_unlock_irqrestore(&desc->lock, flags);

	return 0;
}
EXPORT_SYMBOL(set_irq_chip);

int set_irq_type(unsigned int irq, unsigned int type)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;
	int ret = -ENXIO;

	if (!desc) {
		printk(KERN_ERR "Trying to set irq type for IRQ%d\n", irq);
		return -ENODEV;
	}

	type &= IRQ_TYPE_SENSE_MASK;
	if (type == IRQ_TYPE_NONE)
		return 0;

	raw_spin_lock_irqsave(&desc->lock, flags);
	ret = __irq_set_trigger(desc, irq, type);
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	return ret;
}
EXPORT_SYMBOL(set_irq_type);

int set_irq_data(unsigned int irq, void *data)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc) {
		printk(KERN_ERR
		       "Trying to install controller data for IRQ%d\n", irq);
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&desc->lock, flags);
	desc->handler_data = data;
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	return 0;
}
EXPORT_SYMBOL(set_irq_data);

int set_irq_msi(unsigned int irq, struct msi_desc *entry)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc) {
		printk(KERN_ERR
		       "Trying to install msi data for IRQ%d\n", irq);
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&desc->lock, flags);
	desc->msi_desc = entry;
	if (entry)
		entry->irq = irq;
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	return 0;
}

int set_irq_chip_data(unsigned int irq, void *data)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc) {
		printk(KERN_ERR
		       "Trying to install chip data for IRQ%d\n", irq);
		return -EINVAL;
	}

	if (!desc->chip) {
		printk(KERN_ERR "BUG: bad set_irq_chip_data(IRQ#%d)\n", irq);
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&desc->lock, flags);
	desc->chip_data = data;
	raw_spin_unlock_irqrestore(&desc->lock, flags);

	return 0;
}
EXPORT_SYMBOL(set_irq_chip_data);

void set_irq_nested_thread(unsigned int irq, int nest)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc)
		return;

	raw_spin_lock_irqsave(&desc->lock, flags);
	if (nest)
		desc->status |= IRQ_NESTED_THREAD;
	else
		desc->status &= ~IRQ_NESTED_THREAD;
	raw_spin_unlock_irqrestore(&desc->lock, flags);
}
EXPORT_SYMBOL_GPL(set_irq_nested_thread);

static void default_enable(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->unmask(irq);
	desc->status &= ~IRQ_MASKED;
}

static void default_disable(unsigned int irq)
{
}

static unsigned int default_startup(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->enable(irq);
	return 0;
}

static void default_shutdown(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->mask(irq);
	desc->status |= IRQ_MASKED;
}

void irq_chip_set_defaults(struct irq_chip *chip)
{
	if (!chip->enable)
		chip->enable = default_enable;
	if (!chip->disable)
		chip->disable = default_disable;
	if (!chip->startup)
		chip->startup = default_startup;
	/*
	 * We use chip->disable, when the user provided its own. When
	 * we have default_disable set for chip->disable, then we need
	 * to use default_shutdown, otherwise the irq line is not
	 * disabled on free_irq():
	 */
	if (!chip->shutdown)
		chip->shutdown = chip->disable != default_disable ?
			chip->disable : default_shutdown;
	if (!chip->name)
		chip->name = chip->typename;
	if (!chip->end)
		chip->end = dummy_irq_chip.end;
}

static inline void mask_ack_irq(struct irq_desc *desc, int irq)
{
	if (desc->chip->mask_ack)
		desc->chip->mask_ack(irq);
	else {
		desc->chip->mask(irq);
		if (desc->chip->ack)
			desc->chip->ack(irq);
	}
	desc->status |= IRQ_MASKED;
}

static inline void mask_irq(struct irq_desc *desc, int irq)
{
	if (desc->chip->mask) {
		desc->chip->mask(irq);
		desc->status |= IRQ_MASKED;
	}
}

static inline void unmask_irq(struct irq_desc *desc, int irq)
{
	if (desc->chip->unmask) {
		desc->chip->unmask(irq);
		desc->status &= ~IRQ_MASKED;
	}
}

void handle_nested_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	struct irqaction *action;
	irqreturn_t action_ret;

	might_sleep();

	raw_spin_lock_irq(&desc->lock);

	kstat_incr_irqs_this_cpu(irq, desc);

	action = desc->action;
	if (unlikely(!action || (desc->status & IRQ_DISABLED)))
		goto out_unlock;

	desc->status |= IRQ_INPROGRESS;
	raw_spin_unlock_irq(&desc->lock);

	action_ret = action->thread_fn(action->irq, action->dev_id);
	if (!noirqdebug)
		note_interrupt(irq, desc, action_ret);

	raw_spin_lock_irq(&desc->lock);
	desc->status &= ~IRQ_INPROGRESS;

out_unlock:
	raw_spin_unlock_irq(&desc->lock);
}
EXPORT_SYMBOL_GPL(handle_nested_irq);

void
handle_simple_irq(unsigned int irq, struct irq_desc *desc)
{
	struct irqaction *action;
	irqreturn_t action_ret;

	raw_spin_lock(&desc->lock);

	if (unlikely(desc->status & IRQ_INPROGRESS))
		goto out_unlock;
	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);
	kstat_incr_irqs_this_cpu(irq, desc);

	action = desc->action;
	if (unlikely(!action || (desc->status & IRQ_DISABLED)))
		goto out_unlock;

	desc->status |= IRQ_INPROGRESS;
	raw_spin_unlock(&desc->lock);

	action_ret = handle_IRQ_event(irq, action);
	if (!noirqdebug)
		note_interrupt(irq, desc, action_ret);

	raw_spin_lock(&desc->lock);
	desc->status &= ~IRQ_INPROGRESS;
out_unlock:
	raw_spin_unlock(&desc->lock);
}

void
handle_level_irq(unsigned int irq, struct irq_desc *desc)
{
	struct irqaction *action;
	irqreturn_t action_ret;

//Debug - S
//IT WILL BE REMOVED AFTER WK23
static struct timeval tlast = {0 , 0};
struct timeval t1 = {0 , 0};
struct timeval t2 = {0 , 0};
unsigned long u4TimeDiff;

do_gettimeofday(&t1);
u4TimeDiff = (t1.tv_sec - tlast.tv_sec)*1000000 + t1.tv_usec - tlast.tv_usec;
// 1ms
if(1000 > u4TimeDiff)
{
printk("[IRQDB]irq:%d comes frequently:%lu , status : 0x%x , action:0x%x\n" , irq , u4TimeDiff , desc->status , (unsigned int)desc->action);
}
//Debug - E

	raw_spin_lock(&desc->lock);
	mask_ack_irq(desc, irq);

	if (unlikely(desc->status & IRQ_INPROGRESS))
		goto out_unlock;
	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);
	kstat_incr_irqs_this_cpu(irq, desc);

	/*
	 * If its disabled or no action available
	 * keep it masked and get out of here
	 */
	action = desc->action;
	if (unlikely(!action || (desc->status & IRQ_DISABLED)))
		goto out_unlock;

	desc->status |= IRQ_INPROGRESS;
	raw_spin_unlock(&desc->lock);

	action_ret = handle_IRQ_event(irq, action);
	if (!noirqdebug)
		note_interrupt(irq, desc, action_ret);

	raw_spin_lock(&desc->lock);
	desc->status &= ~IRQ_INPROGRESS;

	if (!(desc->status & (IRQ_DISABLED | IRQ_ONESHOT)))
		unmask_irq(desc, irq);

//Debug - S
//IT WILL BE REMOVED AFTER WK23
do_gettimeofday(&t2);
u4TimeDiff = (t2.tv_sec - t1.tv_sec)*1000000 + t2.tv_usec - t1.tv_usec;
// longer than 2 ms
if(2000 < u4TimeDiff)
{
printk("[IRQDB]irq:%d takes too long:%lu\n" , irq , u4TimeDiff);
}
//Debug - E

out_unlock:
	raw_spin_unlock(&desc->lock);
}
EXPORT_SYMBOL_GPL(handle_level_irq);

void
handle_fasteoi_irq(unsigned int irq, struct irq_desc *desc)
{
	struct irqaction *action;
	irqreturn_t action_ret;

	raw_spin_lock(&desc->lock);

	if (unlikely(desc->status & IRQ_INPROGRESS))
		goto out;

	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);
	kstat_incr_irqs_this_cpu(irq, desc);

	/*
	 * If its disabled or no action available
	 * then mask it and get out of here:
	 */
	action = desc->action;
	if (unlikely(!action || (desc->status & IRQ_DISABLED))) {
		desc->status |= IRQ_PENDING;
		mask_irq(desc, irq);
		goto out;
	}

	desc->status |= IRQ_INPROGRESS;
	desc->status &= ~IRQ_PENDING;
	raw_spin_unlock(&desc->lock);

	action_ret = handle_IRQ_event(irq, action);
	if (!noirqdebug)
		note_interrupt(irq, desc, action_ret);

	raw_spin_lock(&desc->lock);
	desc->status &= ~IRQ_INPROGRESS;
out:
	desc->chip->eoi(irq);

	raw_spin_unlock(&desc->lock);
}

void
handle_edge_irq(unsigned int irq, struct irq_desc *desc)
{
	raw_spin_lock(&desc->lock);

	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);

	/*
	 * If we're currently running this IRQ, or its disabled,
	 * we shouldn't process the IRQ. Mark it pending, handle
	 * the necessary masking and go out
	 */
	if (unlikely((desc->status & (IRQ_INPROGRESS | IRQ_DISABLED)) ||
		    !desc->action)) {
		desc->status |= (IRQ_PENDING | IRQ_MASKED);
		mask_ack_irq(desc, irq);
		goto out_unlock;
	}
	kstat_incr_irqs_this_cpu(irq, desc);

	/* Start handling the irq */
	if (desc->chip->ack)
		desc->chip->ack(irq);

	/* Mark the IRQ currently in progress.*/
	desc->status |= IRQ_INPROGRESS;

	do {
		struct irqaction *action = desc->action;
		irqreturn_t action_ret;

		if (unlikely(!action)) {
			mask_irq(desc, irq);
			goto out_unlock;
		}

		/*
		 * When another irq arrived while we were handling
		 * one, we could have masked the irq.
		 * Renable it, if it was not disabled in meantime.
		 */
		if (unlikely((desc->status &
			       (IRQ_PENDING | IRQ_MASKED | IRQ_DISABLED)) ==
			      (IRQ_PENDING | IRQ_MASKED))) {
			unmask_irq(desc, irq);
		}

		desc->status &= ~IRQ_PENDING;
		raw_spin_unlock(&desc->lock);
		action_ret = handle_IRQ_event(irq, action);
		if (!noirqdebug)
			note_interrupt(irq, desc, action_ret);
		raw_spin_lock(&desc->lock);

	} while ((desc->status & (IRQ_PENDING | IRQ_DISABLED)) == IRQ_PENDING);

	desc->status &= ~IRQ_INPROGRESS;
out_unlock:
	raw_spin_unlock(&desc->lock);
}

void
handle_percpu_irq(unsigned int irq, struct irq_desc *desc)
{
	irqreturn_t action_ret;

	kstat_incr_irqs_this_cpu(irq, desc);

	if (desc->chip->ack)
		desc->chip->ack(irq);

	action_ret = handle_IRQ_event(irq, desc->action);
	if (!noirqdebug)
		note_interrupt(irq, desc, action_ret);

	if (desc->chip->eoi)
		desc->chip->eoi(irq);
}

void
__set_irq_handler(unsigned int irq, irq_flow_handler_t handle, int is_chained,
		  const char *name)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc) {
		printk(KERN_ERR
		       "Trying to install type control for IRQ%d\n", irq);
		return;
	}

	if (!handle)
		handle = handle_bad_irq;
	else if (desc->chip == &no_irq_chip) {
		printk(KERN_WARNING "Trying to install %sinterrupt handler "
		       "for IRQ%d\n", is_chained ? "chained " : "", irq);
		/*
		 * Some ARM implementations install a handler for really dumb
		 * interrupt hardware without setting an irq_chip. This worked
		 * with the ARM no_irq_chip but the check in setup_irq would
		 * prevent us to setup the interrupt at all. Switch it to
		 * dummy_irq_chip for easy transition.
		 */
		desc->chip = &dummy_irq_chip;
	}

	chip_bus_lock(irq, desc);
	raw_spin_lock_irqsave(&desc->lock, flags);

	/* Uninstall? */
	if (handle == handle_bad_irq) {
		if (desc->chip != &no_irq_chip)
			mask_ack_irq(desc, irq);
		desc->status |= IRQ_DISABLED;
		desc->depth = 1;
	}
	desc->handle_irq = handle;
	desc->name = name;

	if (handle != handle_bad_irq && is_chained) {
		desc->status &= ~IRQ_DISABLED;
		desc->status |= IRQ_NOREQUEST | IRQ_NOPROBE;
		desc->depth = 0;
		desc->chip->startup(irq);
	}
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	chip_bus_sync_unlock(irq, desc);
}
EXPORT_SYMBOL_GPL(__set_irq_handler);

void
set_irq_chip_and_handler(unsigned int irq, struct irq_chip *chip,
			 irq_flow_handler_t handle)
{
	set_irq_chip(irq, chip);
	__set_irq_handler(irq, handle, 0, NULL);
}

void
set_irq_chip_and_handler_name(unsigned int irq, struct irq_chip *chip,
			      irq_flow_handler_t handle, const char *name)
{
	set_irq_chip(irq, chip);
	__set_irq_handler(irq, handle, 0, name);
}

void set_irq_noprobe(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc) {
		printk(KERN_ERR "Trying to mark IRQ%d non-probeable\n", irq);
		return;
	}

	raw_spin_lock_irqsave(&desc->lock, flags);
	desc->status |= IRQ_NOPROBE;
	raw_spin_unlock_irqrestore(&desc->lock, flags);
}

void set_irq_probe(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned long flags;

	if (!desc) {
		printk(KERN_ERR "Trying to mark IRQ%d probeable\n", irq);
		return;
	}

	raw_spin_lock_irqsave(&desc->lock, flags);
	desc->status &= ~IRQ_NOPROBE;
	raw_spin_unlock_irqrestore(&desc->lock, flags);
}
