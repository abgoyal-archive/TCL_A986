
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/bootmem.h>

#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/emma/emma2rh.h>

const char *get_system_type(void)
{
#ifdef CONFIG_NEC_MARKEINS
	return "NEC EMMA2RH Mark-eins";
#else
#error  Unknown NEC board
#endif
}

/* [jsun@junsun.net] PMON passes arguments in C main() style */
void __init prom_init(void)
{
	int argc = fw_arg0;
	char **arg = (char **)fw_arg1;
	int i;

	/* if user passes kernel args, ignore the default one */
	if (argc > 1)
		arcs_cmdline[0] = '\0';

	/* arg[0] is "g", the rest is boot parameters */
	for (i = 1; i < argc; i++) {
		if (strlen(arcs_cmdline) + strlen(arg[i] + 1)
		    >= sizeof(arcs_cmdline))
			break;
		strcat(arcs_cmdline, arg[i]);
		strcat(arcs_cmdline, " ");
	}

#ifdef CONFIG_NEC_MARKEINS
	add_memory_region(0, EMMA2RH_RAM_SIZE, BOOT_MEM_RAM);
#else
#error  Unknown NEC board
#endif
}

void __init prom_free_prom_memory(void)
{
}
