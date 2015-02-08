

#include <linux/init.h>

#include <asm/mach-au1x00/au1000.h>
#include <asm/mach-pb1x00/pb1550.h>
#include <asm/mach-db1x00/bcsr.h>

#include "../platform.h"

static int __init pb1550_dev_init(void)
{
	int swapped;

	/* Pb1550, like all others, also has statuschange irqs; however they're
	* wired up on one of the Au1550's shared GPIO201_205 line, which also
	* services the PCMCIA card interrupts.  So we ignore statuschange and
	* use the GPIO201_205 exclusively for card interrupts, since a) pcmcia
	* drivers are used to shared irqs and b) statuschange isn't really use-
	* ful anyway.
	*/
	db1x_register_pcmcia_socket(PCMCIA_ATTR_PHYS_ADDR,
				    PCMCIA_ATTR_PHYS_ADDR + 0x000400000 - 1,
				    PCMCIA_MEM_PHYS_ADDR,
				    PCMCIA_MEM_PHYS_ADDR  + 0x000400000 - 1,
				    PCMCIA_IO_PHYS_ADDR,
				    PCMCIA_IO_PHYS_ADDR   + 0x000010000 - 1,
				    AU1550_GPIO201_205_INT,
				    AU1550_GPIO0_INT,
				    0,
				    0,
				    0);

	db1x_register_pcmcia_socket(PCMCIA_ATTR_PHYS_ADDR + 0x008000000,
				    PCMCIA_ATTR_PHYS_ADDR + 0x008400000 - 1,
				    PCMCIA_MEM_PHYS_ADDR  + 0x008000000,
				    PCMCIA_MEM_PHYS_ADDR  + 0x008400000 - 1,
				    PCMCIA_IO_PHYS_ADDR   + 0x008000000,
				    PCMCIA_IO_PHYS_ADDR   + 0x008010000 - 1,
				    AU1550_GPIO201_205_INT,
				    AU1550_GPIO1_INT,
				    0,
				    0,
				    1);

	swapped = bcsr_read(BCSR_STATUS) & BCSR_STATUS_PB1550_SWAPBOOT;
	db1x_register_norflash(128 * 1024 * 1024, 4, swapped);

	return 0;
}
device_initcall(pb1550_dev_init);
