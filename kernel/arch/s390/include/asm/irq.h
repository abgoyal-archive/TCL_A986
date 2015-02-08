
#ifndef _ASM_IRQ_H
#define _ASM_IRQ_H

#ifdef __KERNEL__
#include <linux/hardirq.h>


enum interruption_class {
	EXTERNAL_INTERRUPT,
	IO_INTERRUPT,

	NR_IRQS,
};

#endif /* __KERNEL__ */
#endif
