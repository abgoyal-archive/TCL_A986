

#include <asm/delay.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/reboot_fixups.h>
#include <asm/msr.h>
#include <linux/cs5535.h>

static void cs5530a_warm_reset(struct pci_dev *dev)
{
	/* writing 1 to the reset control register, 0x44 causes the
	cs5530a to perform a system warm reset */
	pci_write_config_byte(dev, 0x44, 0x1);
	udelay(50); /* shouldn't get here but be safe and spin-a-while */
	return;
}

static void cs5536_warm_reset(struct pci_dev *dev)
{
	/* writing 1 to the LSB of this MSR causes a hard reset */
	wrmsrl(MSR_DIVIL_SOFT_RESET, 1ULL);
	udelay(50); /* shouldn't get here but be safe and spin a while */
}

static void rdc321x_reset(struct pci_dev *dev)
{
	unsigned i;
	/* Voluntary reset the watchdog timer */
	outl(0x80003840, 0xCF8);
	/* Generate a CPU reset on next tick */
	i = inl(0xCFC);
	/* Use the minimum timer resolution */
	i |= 0x1600;
	outl(i, 0xCFC);
	outb(1, 0x92);
}

struct device_fixup {
	unsigned int vendor;
	unsigned int device;
	void (*reboot_fixup)(struct pci_dev *);
};

static const struct device_fixup fixups_table[] = {
{ PCI_VENDOR_ID_CYRIX, PCI_DEVICE_ID_CYRIX_5530_LEGACY, cs5530a_warm_reset },
{ PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_CS5536_ISA, cs5536_warm_reset },
{ PCI_VENDOR_ID_NS, PCI_DEVICE_ID_NS_SC1100_BRIDGE, cs5530a_warm_reset },
{ PCI_VENDOR_ID_RDC, PCI_DEVICE_ID_RDC_R6030, rdc321x_reset },
};

void mach_reboot_fixups(void)
{
	const struct device_fixup *cur;
	struct pci_dev *dev;
	int i;

	/* we can be called from sysrq-B code. In such a case it is
	 * prohibited to dig PCI */
	if (in_interrupt())
		return;

	for (i=0; i < ARRAY_SIZE(fixups_table); i++) {
		cur = &(fixups_table[i]);
		dev = pci_get_device(cur->vendor, cur->device, NULL);
		if (!dev)
			continue;

		cur->reboot_fixup(dev);
		pci_dev_put(dev);
	}
}

