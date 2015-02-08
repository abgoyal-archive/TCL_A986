

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>

#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/common.h>


#define TZIC_INTCNTL	0x0000	/* Control register */
#define TZIC_INTTYPE	0x0004	/* Controller Type register */
#define TZIC_IMPID	0x0008	/* Distributor Implementer Identification */
#define TZIC_PRIOMASK	0x000C	/* Priority Mask Reg */
#define TZIC_SYNCCTRL	0x0010	/* Synchronizer Control register */
#define TZIC_DSMINT	0x0014	/* DSM interrupt Holdoffregister */
#define TZIC_INTSEC0(i)	(0x0080 + ((i) << 2)) /* Interrupt Security Reg 0 */
#define TZIC_ENSET0(i)	(0x0100 + ((i) << 2)) /* Enable Set Reg 0 */
#define TZIC_ENCLEAR0(i) (0x0180 + ((i) << 2)) /* Enable Clear Reg 0 */
#define TZIC_SRCSET0	0x0200	/* Source Set Register 0 */
#define TZIC_SRCCLAR0	0x0280	/* Source Clear Register 0 */
#define TZIC_PRIORITY0	0x0400	/* Priority Register 0 */
#define TZIC_PND0	0x0D00	/* Pending Register 0 */
#define TZIC_HIPND0	0x0D80	/* High Priority Pending Register */
#define TZIC_WAKEUP0(i)	(0x0E00 + ((i) << 2))	/* Wakeup Config Register */
#define TZIC_SWINT	0x0F00	/* Software Interrupt Rigger Register */
#define TZIC_ID0	0x0FD0	/* Indentification Register 0 */

void __iomem *tzic_base; /* Used as irq controller base in entry-macro.S */

static void tzic_mask_irq(unsigned int irq)
{
	int index, off;

	index = irq >> 5;
	off = irq & 0x1F;
	__raw_writel(1 << off, tzic_base + TZIC_ENCLEAR0(index));
}

static void tzic_unmask_irq(unsigned int irq)
{
	int index, off;

	index = irq >> 5;
	off = irq & 0x1F;
	__raw_writel(1 << off, tzic_base + TZIC_ENSET0(index));
}

static unsigned int wakeup_intr[4];

static int tzic_set_wake_irq(unsigned int irq, unsigned int enable)
{
	unsigned int index, off;

	index = irq >> 5;
	off = irq & 0x1F;

	if (index > 3)
		return -EINVAL;

	if (enable)
		wakeup_intr[index] |= (1 << off);
	else
		wakeup_intr[index] &= ~(1 << off);

	return 0;
}

static struct irq_chip mxc_tzic_chip = {
	.name = "MXC_TZIC",
	.ack = tzic_mask_irq,
	.mask = tzic_mask_irq,
	.unmask = tzic_unmask_irq,
	.set_wake = tzic_set_wake_irq,
};

void __init tzic_init_irq(void __iomem *irqbase)
{
	int i;

	tzic_base = irqbase;
	/* put the TZIC into the reset value with
	 * all interrupts disabled
	 */
	i = __raw_readl(tzic_base + TZIC_INTCNTL);

	__raw_writel(0x80010001, tzic_base + TZIC_INTCNTL);
	__raw_writel(0x1f, tzic_base + TZIC_PRIOMASK);
	__raw_writel(0x02, tzic_base + TZIC_SYNCCTRL);

	for (i = 0; i < 4; i++)
		__raw_writel(0xFFFFFFFF, tzic_base + TZIC_INTSEC0(i));

	/* disable all interrupts */
	for (i = 0; i < 4; i++)
		__raw_writel(0xFFFFFFFF, tzic_base + TZIC_ENCLEAR0(i));

	/* all IRQ no FIQ Warning :: No selection */

	for (i = 0; i < MXC_INTERNAL_IRQS; i++) {
		set_irq_chip(i, &mxc_tzic_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	mxc_register_gpios();

	pr_info("TrustZone Interrupt Controller (TZIC) initialized\n");
}

int tzic_enable_wake(int is_idle)
{
	unsigned int i, v;

	__raw_writel(1, tzic_base + TZIC_DSMINT);
	if (unlikely(__raw_readl(tzic_base + TZIC_DSMINT) == 0))
		return -EAGAIN;

	for (i = 0; i < 4; i++) {
		v = is_idle ? __raw_readl(TZIC_ENSET0(i)) : wakeup_intr[i];
		__raw_writel(v, TZIC_WAKEUP0(i));
	}

	return 0;
}
