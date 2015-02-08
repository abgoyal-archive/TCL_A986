

#include <linux/types.h>
#include <linux/ssb/ssb.h>
#include <linux/ssb/ssb_embedded.h>
#include <asm/bootinfo.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <bcm47xx.h>
#include <asm/fw/cfe/cfe_api.h>
#include <asm/mach-bcm47xx/nvram.h>

struct ssb_bus ssb_bcm47xx;
EXPORT_SYMBOL(ssb_bcm47xx);

static void bcm47xx_machine_restart(char *command)
{
	printk(KERN_ALERT "Please stand by while rebooting the system...\n");
	local_irq_disable();
	/* Set the watchdog timer to reset immediately */
	ssb_watchdog_timer_set(&ssb_bcm47xx, 1);
	while (1)
		cpu_relax();
}

static void bcm47xx_machine_halt(void)
{
	/* Disable interrupts and watchdog and spin forever */
	local_irq_disable();
	ssb_watchdog_timer_set(&ssb_bcm47xx, 0);
	while (1)
		cpu_relax();
}

static void str2eaddr(char *str, char *dest)
{
	int i = 0;

	if (str == NULL) {
		memset(dest, 0, 6);
		return;
	}

	for (;;) {
		dest[i++] = (char) simple_strtoul(str, NULL, 16);
		str += 2;
		if (!*str++ || i == 6)
			break;
	}
}

static int bcm47xx_get_invariants(struct ssb_bus *bus,
				   struct ssb_init_invariants *iv)
{
	char buf[100];

	/* Fill boardinfo structure */
	memset(&(iv->boardinfo), 0 , sizeof(struct ssb_boardinfo));

	if (cfe_getenv("boardvendor", buf, sizeof(buf)) >= 0 ||
	    nvram_getenv("boardvendor", buf, sizeof(buf)) >= 0)
		iv->boardinfo.type = (u16)simple_strtoul(buf, NULL, 0);
	if (cfe_getenv("boardtype", buf, sizeof(buf)) >= 0 ||
	    nvram_getenv("boardtype", buf, sizeof(buf)) >= 0)
		iv->boardinfo.type = (u16)simple_strtoul(buf, NULL, 0);
	if (cfe_getenv("boardrev", buf, sizeof(buf)) >= 0 ||
	    nvram_getenv("boardrev", buf, sizeof(buf)) >= 0)
		iv->boardinfo.rev = (u16)simple_strtoul(buf, NULL, 0);

	/* Fill sprom structure */
	memset(&(iv->sprom), 0, sizeof(struct ssb_sprom));
	iv->sprom.revision = 3;

	if (cfe_getenv("et0macaddr", buf, sizeof(buf)) >= 0 ||
	    nvram_getenv("et0macaddr", buf, sizeof(buf)) >= 0)
		str2eaddr(buf, iv->sprom.et0mac);

	if (cfe_getenv("et1macaddr", buf, sizeof(buf)) >= 0 ||
	    nvram_getenv("et1macaddr", buf, sizeof(buf)) >= 0)
		str2eaddr(buf, iv->sprom.et1mac);

	if (cfe_getenv("et0phyaddr", buf, sizeof(buf)) >= 0 ||
	    nvram_getenv("et0phyaddr", buf, sizeof(buf)) >= 0)
		iv->sprom.et0phyaddr = simple_strtoul(buf, NULL, 0);

	if (cfe_getenv("et1phyaddr", buf, sizeof(buf)) >= 0 ||
	    nvram_getenv("et1phyaddr", buf, sizeof(buf)) >= 0)
		iv->sprom.et1phyaddr = simple_strtoul(buf, NULL, 0);

	if (cfe_getenv("et0mdcport", buf, sizeof(buf)) >= 0 ||
	    nvram_getenv("et0mdcport", buf, sizeof(buf)) >= 0)
		iv->sprom.et0mdcport = simple_strtoul(buf, NULL, 10);

	if (cfe_getenv("et1mdcport", buf, sizeof(buf)) >= 0 ||
	    nvram_getenv("et1mdcport", buf, sizeof(buf)) >= 0)
		iv->sprom.et1mdcport = simple_strtoul(buf, NULL, 10);

	return 0;
}

void __init plat_mem_setup(void)
{
	int err;

	err = ssb_bus_ssbbus_register(&ssb_bcm47xx, SSB_ENUM_BASE,
				      bcm47xx_get_invariants);
	if (err)
		panic("Failed to initialize SSB bus (err %d)\n", err);

	_machine_restart = bcm47xx_machine_restart;
	_machine_halt = bcm47xx_machine_halt;
	pm_power_off = bcm47xx_machine_halt;
}
