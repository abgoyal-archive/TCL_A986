
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <asm/machvec.h>
#include <mach/snapgear.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <cpu/timer.h>


static irqreturn_t eraseconfig_interrupt(int irq, void *dev_id)
{
	(void)__raw_readb(0xb8000000);	/* dummy read */

	printk("SnapGear: erase switch interrupt!\n");

	return IRQ_HANDLED;
}

static int __init eraseconfig_init(void)
{
	printk("SnapGear: EraseConfig init\n");
	/* Setup "EraseConfig" switch on external IRQ 0 */
	if (request_irq(IRL0_IRQ, eraseconfig_interrupt, IRQF_DISABLED,
				"Erase Config", NULL))
		printk("SnapGear: failed to register IRQ%d for Reset witch\n",
				IRL0_IRQ);
	else
		printk("SnapGear: registered EraseConfig switch on IRQ%d\n",
				IRL0_IRQ);
	return(0);
}

module_init(eraseconfig_init);

/****************************************************************************/

static void __init init_snapgear_IRQ(void)
{
	printk("Setup SnapGear IRQ/IPR ...\n");
	/* enable individual interrupt mode for externals */
	plat_irq_setup_pins(IRQ_MODE_IRQ);
}

static struct sh_machine_vector mv_snapgear __initmv = {
	.mv_name		= "SnapGear SecureEdge5410",
	.mv_nr_irqs		= 72,

	.mv_inb			= snapgear_inb,
	.mv_inw			= snapgear_inw,
	.mv_inl			= snapgear_inl,
	.mv_outb		= snapgear_outb,
	.mv_outw		= snapgear_outw,
	.mv_outl		= snapgear_outl,

	.mv_inb_p		= snapgear_inb_p,
	.mv_inw_p		= snapgear_inw,
	.mv_inl_p		= snapgear_inl,
	.mv_outb_p		= snapgear_outb_p,
	.mv_outw_p		= snapgear_outw,
	.mv_outl_p		= snapgear_outl,

	.mv_init_irq		= init_snapgear_IRQ,
};
