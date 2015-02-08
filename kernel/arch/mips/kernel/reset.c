
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/types.h>
#include <linux/reboot.h>

#include <asm/reboot.h>

void (*_machine_restart)(char *command);
void (*_machine_halt)(void);
void (*pm_power_off)(void);

EXPORT_SYMBOL(pm_power_off);

void machine_restart(char *command)
{
	if (_machine_restart)
		_machine_restart(command);
}

void machine_halt(void)
{
	if (_machine_halt)
		_machine_halt();
}

void machine_power_off(void)
{
	if (pm_power_off)
		pm_power_off();
}
