

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

#include <mach/irqs-eb.h>
#include <mach/irqs-pb11mp.h>
#include <mach/irqs-pb1176.h>
#include <mach/irqs-pba8.h>
#include <mach/irqs-pbx.h>

#define IRQ_LOCALTIMER		29
#define IRQ_LOCALWDOG		30

#define IRQ_GIC_START		32

#ifndef NR_IRQS
#error "NR_IRQS not defined by the board-specific files"
#endif

#endif
