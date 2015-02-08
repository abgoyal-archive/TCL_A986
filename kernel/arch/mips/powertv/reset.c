
#include <linux/pm.h>

#include <linux/io.h>
#include <asm/reboot.h>			/* Not included by linux/reboot.h */

#ifdef CONFIG_BOOTLOADER_DRIVER
#include <asm/mach-powertv/kbldr.h>
#endif

#include <asm/mach-powertv/asic_regs.h>
#include "reset.h"

static void mips_machine_restart(char *command)
{
#ifdef CONFIG_BOOTLOADER_DRIVER
	/*
	 * Call the bootloader's reset function to ensure
	 * that persistent data is flushed before hard reset
	 */
	kbldr_SetCauseAndReset();
#else
	writel(0x1, asic_reg_addr(watchdog));
#endif
}

void mips_reboot_setup(void)
{
	_machine_restart = mips_machine_restart;
}
