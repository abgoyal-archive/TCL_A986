

#ifndef __MACH_IRQS_H
#define __MACH_IRQS_H

#define IRQ_LOCALTIMER		29
#define IRQ_LOCALWDOG		30
#define IRQ_TC11MP_GIC_START	32

#include <mach/cns3xxx.h>

#ifndef NR_IRQS
#error "NR_IRQS not defined by the board-specific files"
#endif

#endif
