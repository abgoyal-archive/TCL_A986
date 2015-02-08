
#include <linux/reboot.h>
#include <pnx833x.h>

void pnx833x_machine_restart(char *command)
{
	PNX833X_RESET_CONTROL_2 = 0;
	PNX833X_RESET_CONTROL = 0;
}

void pnx833x_machine_halt(void)
{
	while (1)
		__asm__ __volatile__ ("wait");

}

void pnx833x_machine_power_off(void)
{
	pnx833x_machine_halt();
}
