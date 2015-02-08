

#include <linux/init.h>
#include <asm/mach-au1x00/au1000.h>
#include <asm/mach-db1x00/bcsr.h>

#include "../platform.h"

static int __init pb1500_dev_init(void)
{
	int swapped;

	/* PCMCIA. single socket, identical to Pb1500 */
	db1x_register_pcmcia_socket(PCMCIA_ATTR_PHYS_ADDR,
				    PCMCIA_ATTR_PHYS_ADDR + 0x000400000 - 1,
				    PCMCIA_MEM_PHYS_ADDR,
				    PCMCIA_MEM_PHYS_ADDR  + 0x000400000 - 1,
				    PCMCIA_IO_PHYS_ADDR,
				    PCMCIA_IO_PHYS_ADDR   + 0x000010000 - 1,
				    AU1500_GPIO11_INT,	 /* card */
				    AU1500_GPIO9_INT,	 /* insert */
				    /*AU1500_GPIO10_INT*/0, /* stschg */
				    0,			 /* eject */
				    0);			 /* id */

	swapped = bcsr_read(BCSR_STATUS) &  BCSR_STATUS_DB1000_SWAPBOOT;
	db1x_register_norflash(64 * 1024 * 1024, 4, swapped);

	return 0;
}
device_initcall(pb1500_dev_init);
