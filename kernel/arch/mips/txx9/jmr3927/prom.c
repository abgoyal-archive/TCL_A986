
#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/bootinfo.h>
#include <asm/txx9/generic.h>
#include <asm/txx9/jmr3927.h>

void __init jmr3927_prom_init(void)
{
	/* CCFG */
	if ((tx3927_ccfgptr->ccfg & TX3927_CCFG_TLBOFF) == 0)
		printk(KERN_ERR "TX3927 TLB off\n");

	add_memory_region(0, JMR3927_SDRAM_SIZE, BOOT_MEM_RAM);
	txx9_sio_putchar_init(TX3927_SIO_REG(1));
}
