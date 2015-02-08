

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/bootmem.h>
#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <linux/string.h>
#include <linux/kernel.h>

int prom_argc;
char **prom_argv, **prom_envp;
extern void  __init prom_init_cmdline(void);
extern char *prom_getenv(char *envname);

const char *get_system_type(void)
{
	return "NXP PNX8550/JBS";
}

void __init prom_init(void)
{
	unsigned long memsize;

	//memsize = 0x02800000; /* Trimedia uses memory above */
	memsize = 0x08000000; /* Trimedia uses memory above */
	add_memory_region(0, memsize, BOOT_MEM_RAM);
}
