

#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/bootinfo.h>
#include <asm/mach-au1x00/au1000.h>

#include <prom.h>

const char *get_system_type(void)
{
	return "MTX-1";
}

void __init prom_init(void)
{
	unsigned char *memsize_str;
	unsigned long memsize;

	prom_argc = fw_arg0;
	prom_argv = (char **)fw_arg1;
	prom_envp = (char **)fw_arg2;

	prom_init_cmdline();

	memsize_str = prom_getenv("memsize");
	if (!memsize_str)
		memsize = 0x04000000;
	else
		strict_strtoul(memsize_str, 0, &memsize);
	add_memory_region(0, memsize, BOOT_MEM_RAM);
}

void prom_putchar(unsigned char c)
{
	alchemy_uart_putchar(UART0_PHYS_ADDR, c);
}
