

#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/bootinfo.h>
#include <asm/mach-au1x00/au1000.h>
#include <prom.h>

#if defined(CONFIG_MIPS_PB1000) || defined(CONFIG_MIPS_DB1000) || \
    defined(CONFIG_MIPS_PB1100) || defined(CONFIG_MIPS_DB1100) || \
    defined(CONFIG_MIPS_PB1500) || defined(CONFIG_MIPS_DB1500) || \
    defined(CONFIG_MIPS_BOSPORUS) || defined(CONFIG_MIPS_MIRAGE)
#define ALCHEMY_BOARD_DEFAULT_MEMSIZE	0x04000000

#else	/* Au1550/Au1200-based develboards */
#define ALCHEMY_BOARD_DEFAULT_MEMSIZE	0x08000000
#endif

void __init prom_init(void)
{
	unsigned char *memsize_str;
	unsigned long memsize;

	prom_argc = (int)fw_arg0;
	prom_argv = (char **)fw_arg1;
	prom_envp = (char **)fw_arg2;

	prom_init_cmdline();
	memsize_str = prom_getenv("memsize");
	if (!memsize_str)
		memsize = ALCHEMY_BOARD_DEFAULT_MEMSIZE;
	else
		strict_strtoul(memsize_str, 0, &memsize);
	add_memory_region(0, memsize, BOOT_MEM_RAM);
}

void prom_putchar(unsigned char c)
{
    alchemy_uart_putchar(UART0_PHYS_ADDR, c);
}
