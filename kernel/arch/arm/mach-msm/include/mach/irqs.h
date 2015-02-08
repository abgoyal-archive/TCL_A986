

#ifndef __ASM_ARCH_MSM_IRQS_H
#define __ASM_ARCH_MSM_IRQS_H

#define MSM_IRQ_BIT(irq)     (1 << ((irq) & 31))

#if defined(CONFIG_ARCH_MSM7X30)
#include "irqs-7x30.h"
#elif defined(CONFIG_ARCH_QSD8X50)
#include "irqs-8x50.h"
#include "sirc.h"
#elif defined(CONFIG_ARCH_MSM_ARM11)
#include "irqs-7x00.h"
#else
#error "Unknown architecture specification"
#endif

#define NR_IRQS (NR_MSM_IRQS + NR_GPIO_IRQS + NR_BOARD_IRQS)
#define MSM_GPIO_TO_INT(n) (NR_MSM_IRQS + (n))
#define MSM_INT_TO_REG(base, irq) (base + irq / 32)

#endif
