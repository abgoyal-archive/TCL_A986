

#include <linux/sched.h>
#include <linux/irq.h>

static unsigned int
lsapic_noop_startup (unsigned int irq)
{
	return 0;
}

static void
lsapic_noop (unsigned int irq)
{
	/* nothing to do... */
}

static int lsapic_retrigger(unsigned int irq)
{
	ia64_resend_irq(irq);

	return 1;
}

struct irq_chip irq_type_ia64_lsapic = {
	.name =		"LSAPIC",
	.startup =	lsapic_noop_startup,
	.shutdown =	lsapic_noop,
	.enable =	lsapic_noop,
	.disable =	lsapic_noop,
	.ack =		lsapic_noop,
	.end =		lsapic_noop,
	.retrigger =	lsapic_retrigger,
};
