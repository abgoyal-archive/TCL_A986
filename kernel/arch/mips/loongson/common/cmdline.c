
#include <asm/bootinfo.h>

#include <loongson.h>

void __init prom_init_cmdline(void)
{
	int prom_argc;
	/* pmon passes arguments in 32bit pointers */
	int *_prom_argv;
	int i;
	long l;

	/* firmware arguments are initialized in head.S */
	prom_argc = fw_arg0;
	_prom_argv = (int *)fw_arg1;

	/* arg[0] is "g", the rest is boot parameters */
	arcs_cmdline[0] = '\0';
	for (i = 1; i < prom_argc; i++) {
		l = (long)_prom_argv[i];
		if (strlen(arcs_cmdline) + strlen(((char *)l) + 1)
		    >= sizeof(arcs_cmdline))
			break;
		strcat(arcs_cmdline, ((char *)l));
		strcat(arcs_cmdline, " ");
	}

	if ((strstr(arcs_cmdline, "console=")) == NULL)
		strcat(arcs_cmdline, " console=ttyS0,115200");
	if ((strstr(arcs_cmdline, "root=")) == NULL)
		strcat(arcs_cmdline, " root=/dev/hda1");

	prom_init_machtype();
}
