

#include <linux/acpi_pmtmr.h>
#include <linux/clocksource.h>
#include <linux/timex.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <asm/io.h>

u32 pmtmr_ioport __read_mostly;

static inline u32 read_pmtmr(void)
{
	/* mask the output to 24 bits */
	return inl(pmtmr_ioport) & ACPI_PM_MASK;
}

u32 acpi_pm_read_verified(void)
{
	u32 v1 = 0, v2 = 0, v3 = 0;

	/*
	 * It has been reported that because of various broken
	 * chipsets (ICH4, PIIX4 and PIIX4E) where the ACPI PM clock
	 * source is not latched, you must read it multiple
	 * times to ensure a safe value is read:
	 */
	do {
		v1 = read_pmtmr();
		v2 = read_pmtmr();
		v3 = read_pmtmr();
	} while (unlikely((v1 > v2 && v1 < v3) || (v2 > v3 && v2 < v1)
			  || (v3 > v1 && v3 < v2)));

	return v2;
}

static cycle_t acpi_pm_read(struct clocksource *cs)
{
	return (cycle_t)read_pmtmr();
}

static struct clocksource clocksource_acpi_pm = {
	.name		= "acpi_pm",
	.rating		= 200,
	.read		= acpi_pm_read,
	.mask		= (cycle_t)ACPI_PM_MASK,
	.mult		= 0, /*to be calculated*/
	.shift		= 22,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,

};


#ifdef CONFIG_PCI
static int __devinitdata acpi_pm_good;
static int __init acpi_pm_good_setup(char *__str)
{
	acpi_pm_good = 1;
	return 1;
}
__setup("acpi_pm_good", acpi_pm_good_setup);

static cycle_t acpi_pm_read_slow(struct clocksource *cs)
{
	return (cycle_t)acpi_pm_read_verified();
}

static inline void acpi_pm_need_workaround(void)
{
	clocksource_acpi_pm.read = acpi_pm_read_slow;
	clocksource_acpi_pm.rating = 120;
}

static void __devinit acpi_pm_check_blacklist(struct pci_dev *dev)
{
	if (acpi_pm_good)
		return;

	/* the bug has been fixed in PIIX4M */
	if (dev->revision < 3) {
		printk(KERN_WARNING "* Found PM-Timer Bug on the chipset."
		       " Due to workarounds for a bug,\n"
		       "* this clock source is slow. Consider trying"
		       " other clock sources\n");

		acpi_pm_need_workaround();
	}
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82371AB_3,
			acpi_pm_check_blacklist);

static void __devinit acpi_pm_check_graylist(struct pci_dev *dev)
{
	if (acpi_pm_good)
		return;

	printk(KERN_WARNING "* The chipset may have PM-Timer Bug. Due to"
	       " workarounds for a bug,\n"
	       "* this clock source is slow. If you are sure your timer"
	       " does not have\n"
	       "* this bug, please use \"acpi_pm_good\" to disable the"
	       " workaround\n");

	acpi_pm_need_workaround();
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82801DB_0,
			acpi_pm_check_graylist);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_SERVERWORKS, PCI_DEVICE_ID_SERVERWORKS_LE,
			acpi_pm_check_graylist);
#endif

#ifndef CONFIG_X86_64
#include <asm/mach_timer.h>
#define PMTMR_EXPECTED_RATE \
  ((CALIBRATE_LATCH * (PMTMR_TICKS_PER_SEC >> 10)) / (CLOCK_TICK_RATE>>10))
static int verify_pmtmr_rate(void)
{
	cycle_t value1, value2;
	unsigned long count, delta;

	mach_prepare_counter();
	value1 = clocksource_acpi_pm.read(&clocksource_acpi_pm);
	mach_countup(&count);
	value2 = clocksource_acpi_pm.read(&clocksource_acpi_pm);
	delta = (value2 - value1) & ACPI_PM_MASK;

	/* Check that the PMTMR delta is within 5% of what we expect */
	if (delta < (PMTMR_EXPECTED_RATE * 19) / 20 ||
	    delta > (PMTMR_EXPECTED_RATE * 21) / 20) {
		printk(KERN_INFO "PM-Timer running at invalid rate: %lu%% "
			"of normal - aborting.\n",
			100UL * delta / PMTMR_EXPECTED_RATE);
		return -1;
	}

	return 0;
}
#else
#define verify_pmtmr_rate() (0)
#endif

/* Number of monotonicity checks to perform during initialization */
#define ACPI_PM_MONOTONICITY_CHECKS 10
/* Number of reads we try to get two different values */
#define ACPI_PM_READ_CHECKS 10000

static int __init init_acpi_pm_clocksource(void)
{
	cycle_t value1, value2;
	unsigned int i, j = 0;

	if (!pmtmr_ioport)
		return -ENODEV;

	clocksource_acpi_pm.mult = clocksource_hz2mult(PMTMR_TICKS_PER_SEC,
						clocksource_acpi_pm.shift);

	/* "verify" this timing source: */
	for (j = 0; j < ACPI_PM_MONOTONICITY_CHECKS; j++) {
		udelay(100 * j);
		value1 = clocksource_acpi_pm.read(&clocksource_acpi_pm);
		for (i = 0; i < ACPI_PM_READ_CHECKS; i++) {
			value2 = clocksource_acpi_pm.read(&clocksource_acpi_pm);
			if (value2 == value1)
				continue;
			if (value2 > value1)
				break;
			if ((value2 < value1) && ((value2) < 0xFFF))
				break;
			printk(KERN_INFO "PM-Timer had inconsistent results:"
			       " 0x%#llx, 0x%#llx - aborting.\n",
			       value1, value2);
			return -EINVAL;
		}
		if (i == ACPI_PM_READ_CHECKS) {
			printk(KERN_INFO "PM-Timer failed consistency check "
			       " (0x%#llx) - aborting.\n", value1);
			return -ENODEV;
		}
	}

	if (verify_pmtmr_rate() != 0)
		return -ENODEV;

	return clocksource_register(&clocksource_acpi_pm);
}

fs_initcall(init_acpi_pm_clocksource);

static int __init parse_pmtmr(char *arg)
{
	unsigned long base;

	if (strict_strtoul(arg, 16, &base))
		return -EINVAL;
#ifdef CONFIG_X86_64
	if (base > UINT_MAX)
		return -ERANGE;
#endif
	printk(KERN_INFO "PMTMR IOPort override: 0x%04x -> 0x%04lx\n",
	       pmtmr_ioport, base);
	pmtmr_ioport = base;

	return 1;
}
__setup("pmtmr=", parse_pmtmr);
