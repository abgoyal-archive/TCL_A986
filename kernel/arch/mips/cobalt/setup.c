
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/pm.h>

#include <asm/bootinfo.h>
#include <asm/reboot.h>
#include <asm/gt64120.h>

#include <cobalt.h>

extern void cobalt_machine_restart(char *command);
extern void cobalt_machine_halt(void);

const char *get_system_type(void)
{
	switch (cobalt_board_id) {
		case COBALT_BRD_ID_QUBE1:
			return "Cobalt Qube";
		case COBALT_BRD_ID_RAQ1:
			return "Cobalt RaQ";
		case COBALT_BRD_ID_QUBE2:
			return "Cobalt Qube2";
		case COBALT_BRD_ID_RAQ2:
			return "Cobalt RaQ2";
	}
	return "MIPS Cobalt";
}

static struct resource cobalt_reserved_resources[] = {
	{	/* dma1 */
		.start	= 0x00,
		.end	= 0x1f,
		.name	= "reserved",
		.flags	= IORESOURCE_BUSY | IORESOURCE_IO,
	},
	{	/* keyboard */
		.start	= 0x60,
		.end	= 0x6f,
		.name	= "reserved",
		.flags	= IORESOURCE_BUSY | IORESOURCE_IO,
	},
	{	/* dma page reg */
		.start	= 0x80,
		.end	= 0x8f,
		.name	= "reserved",
		.flags	= IORESOURCE_BUSY | IORESOURCE_IO,
	},
	{	/* dma2 */
		.start	= 0xc0,
		.end	= 0xdf,
		.name	= "reserved",
		.flags	= IORESOURCE_BUSY | IORESOURCE_IO,
	},
};

void __init plat_mem_setup(void)
{
	int i;

	_machine_restart = cobalt_machine_restart;
	_machine_halt = cobalt_machine_halt;
	pm_power_off = cobalt_machine_halt;

	set_io_port_base(CKSEG1ADDR(GT_DEF_PCI0_IO_BASE));

	/* I/O port resource */
	ioport_resource.end = 0x01ffffff;

	/* These resources have been reserved by VIA SuperI/O chip. */
	for (i = 0; i < ARRAY_SIZE(cobalt_reserved_resources); i++)
		request_resource(&ioport_resource, cobalt_reserved_resources + i);
}


void __init prom_init(void)
{
	unsigned long memsz;
	int argc, i;
	char **argv;

	memsz = fw_arg0 & 0x7fff0000;
	argc = fw_arg0 & 0x0000ffff;
	argv = (char **)fw_arg1;

	for (i = 1; i < argc; i++) {
		strlcat(arcs_cmdline, argv[i], COMMAND_LINE_SIZE);
		if (i < (argc - 1))
			strlcat(arcs_cmdline, " ", COMMAND_LINE_SIZE);
	}

	add_memory_region(0x0, memsz, BOOT_MEM_RAM);
}

void __init prom_free_prom_memory(void)
{
	/* Nothing to do! */
}
