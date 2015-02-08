

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <asm/openprom.h>
#include <asm/oplib.h>

extern void restore_current(void);

void
prom_putsegment(int ctx, unsigned long vaddr, int segment)
{
	unsigned long flags;
	spin_lock_irqsave(&prom_lock, flags);
	(*(romvec->pv_setctxt))(ctx, (char *) vaddr, segment);
	restore_current();
	spin_unlock_irqrestore(&prom_lock, flags);
}
