

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <mach/bridge-regs.h>
#include <plat/irq.h>
#include "common.h"

void __init loki_init_irq(void)
{
	orion_irq_init(0, (void __iomem *)(IRQ_VIRT_BASE + IRQ_MASK_OFF));
}
