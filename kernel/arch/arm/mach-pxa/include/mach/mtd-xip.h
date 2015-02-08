

#ifndef __ARCH_PXA_MTD_XIP_H__
#define __ARCH_PXA_MTD_XIP_H__

#include <mach/regs-ost.h>
#include <mach/regs-intc.h>

#define xip_irqpending()	(ICIP & ICMR)

/* we sample OSCR and convert desired delta to usec (1/4 ~= 1000000/3686400) */
#define xip_currtime()		(OSCR)
#define xip_elapsed_since(x)	(signed)((OSCR - (x)) / 4)


#define xip_cpu_idle()  asm volatile ("mcr p14, 0, %0, c7, c0, 0" :: "r" (1))

#endif /* __ARCH_PXA_MTD_XIP_H__ */
