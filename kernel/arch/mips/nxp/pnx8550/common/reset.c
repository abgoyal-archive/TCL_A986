
#include <linux/kernel.h>

#include <asm/reboot.h>
#include <glb.h>

void pnx8550_machine_restart(char *command)
{
	char head[] = "************* Machine restart *************";
	char foot[] = "*******************************************";

	printk("\n\n");
	printk("%s\n", head);
	if (command != NULL)
		printk("* %s\n", command);
	printk("%s\n", foot);

	PNX8550_RST_CTL = PNX8550_RST_DO_SW_RST;
}

void pnx8550_machine_halt(void)
{
	printk("*** Machine halt. (Not implemented) ***\n");
}

void pnx8550_machine_power_off(void)
{
	printk("*** Machine power off.  (Not implemented) ***\n");
}
