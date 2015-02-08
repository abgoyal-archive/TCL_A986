

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/module.h>

#include <asm/mipsregs.h>
#include <asm/time.h>

#include <au1000.h>

extern void __init board_setup(void);
extern void set_cpuspec(void);

void __init plat_mem_setup(void)
{
	unsigned long est_freq;

	/* determine core clock */
	est_freq = au1xxx_calc_clock();
	est_freq += 5000;    /* round */
	est_freq -= est_freq % 10000;
	printk(KERN_INFO "(PRId %08x) @ %lu.%02lu MHz\n", read_c0_prid(),
	       est_freq / 1000000, ((est_freq % 1000000) * 100) / 1000000);

	/* this is faster than wasting cycles trying to approximate it */
	preset_lpj = (est_freq >> 1) / HZ;

	board_setup();  /* board specific setup */

	if (au1xxx_cpu_needs_config_od())
		/* Various early Au1xx0 errata corrected by this */
		set_c0_config(1 << 19); /* Set Config[OD] */
	else
		/* Clear to obtain best system bus performance */
		clear_c0_config(1 << 19); /* Clear Config[OD] */

	/* IO/MEM resources. */
	set_io_port_base(0);
	ioport_resource.start = IOPORT_RESOURCE_START;
	ioport_resource.end = IOPORT_RESOURCE_END;
	iomem_resource.start = IOMEM_RESOURCE_START;
	iomem_resource.end = IOMEM_RESOURCE_END;
}

#if defined(CONFIG_64BIT_PHYS_ADDR) && defined(CONFIG_PCI)
/* This routine should be valid for all Au1x based boards */
phys_t __fixup_bigphys_addr(phys_t phys_addr, phys_t size)
{
	u32 start = (u32)Au1500_PCI_MEM_START;
	u32 end   = (u32)Au1500_PCI_MEM_END;

	/* Don't fixup 36-bit addresses */
	if ((phys_addr >> 32) != 0)
		return phys_addr;

	/* Check for PCI memory window */
	if (phys_addr >= start && (phys_addr + size - 1) <= end)
		return (phys_t)((phys_addr - start) + Au1500_PCI_MEM_START);

	/* default nop */
	return phys_addr;
}
EXPORT_SYMBOL(__fixup_bigphys_addr);
#endif
