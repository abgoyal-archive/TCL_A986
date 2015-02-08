
#include <linux/init.h>
#include <asm/machvec.h>
#include <mach/systemh7751.h>

extern void make_systemh_irq(unsigned int irq);

static void __init sh7751systemh_init_irq(void)
{
	make_systemh_irq(0xb);	/* Ethernet interrupt */
}

static struct sh_machine_vector mv_7751systemh __initmv = {
	.mv_name		= "7751 SystemH",
	.mv_nr_irqs		= 72,

	.mv_inb			= sh7751systemh_inb,
	.mv_inw			= sh7751systemh_inw,
	.mv_inl			= sh7751systemh_inl,
	.mv_outb		= sh7751systemh_outb,
	.mv_outw		= sh7751systemh_outw,
	.mv_outl		= sh7751systemh_outl,

	.mv_inb_p		= sh7751systemh_inb_p,
	.mv_inw_p		= sh7751systemh_inw,
	.mv_inl_p		= sh7751systemh_inl,
	.mv_outb_p		= sh7751systemh_outb_p,
	.mv_outw_p		= sh7751systemh_outw,
	.mv_outl_p		= sh7751systemh_outl,

	.mv_insb		= sh7751systemh_insb,
	.mv_insw		= sh7751systemh_insw,
	.mv_insl		= sh7751systemh_insl,
	.mv_outsb		= sh7751systemh_outsb,
	.mv_outsw		= sh7751systemh_outsw,
	.mv_outsl		= sh7751systemh_outsl,

	.mv_init_irq		= sh7751systemh_init_irq,
};
