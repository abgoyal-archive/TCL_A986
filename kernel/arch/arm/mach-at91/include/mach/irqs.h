

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

#include <linux/io.h>
#include <mach/at91_aic.h>

#define NR_AIC_IRQS 32


#define irq_finish(irq) do { at91_sys_write(AT91_AIC_EOICR, 0); } while (0)


#define	NR_IRQS		(NR_AIC_IRQS + (5 * 32))

/* FIQ is AIC source 0. */
#define FIQ_START AT91_ID_FIQ

#endif
