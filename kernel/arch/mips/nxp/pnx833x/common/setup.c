
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <asm/reboot.h>
#include <pnx833x.h>
#include <gpio.h>

extern void pnx833x_board_setup(void);
extern void pnx833x_machine_restart(char *);
extern void pnx833x_machine_halt(void);
extern void pnx833x_machine_power_off(void);

int __init plat_mem_setup(void)
{
	/* fake pci bus to avoid bounce buffers */
	PCI_DMA_BUS_IS_PHYS = 1;

	/* set mips clock to 320MHz */
#if defined(CONFIG_SOC_PNX8335)
	PNX8335_WRITEFIELD(0x17, CLOCK_PLL_CPU_CTL, FREQ);
#endif
	pnx833x_gpio_init();	/* so it will be ready in board_setup() */

	pnx833x_board_setup();

	_machine_restart = pnx833x_machine_restart;
	_machine_halt = pnx833x_machine_halt;
	pm_power_off = pnx833x_machine_power_off;

	/* IO/MEM resources. */
	set_io_port_base(KSEG1);
	ioport_resource.start = 0;
	ioport_resource.end = ~0;
	iomem_resource.start = 0;
	iomem_resource.end = ~0;

	return 0;
}
