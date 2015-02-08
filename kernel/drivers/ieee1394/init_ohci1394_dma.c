

#include <linux/interrupt.h>	/* for ohci1394.h */
#include <linux/delay.h>
#include <linux/pci.h>		/* for PCI defines */
#include <linux/init_ohci1394_dma.h>
#include <asm/pci-direct.h>	/* for direct PCI config space access */
#include <asm/fixmap.h>

#include "ieee1394_types.h"
#include "ohci1394.h"

int __initdata init_ohci1394_dma_early;

/* Reads a PHY register of an OHCI-1394 controller */
static inline u8 __init get_phy_reg(struct ti_ohci *ohci, u8 addr)
{
	int i;
	quadlet_t r;

	reg_write(ohci, OHCI1394_PhyControl, (addr << 8) | 0x00008000);

	for (i = 0; i < OHCI_LOOP_COUNT; i++) {
		if (reg_read(ohci, OHCI1394_PhyControl) & 0x80000000)
			break;
		mdelay(1);
	}
	r = reg_read(ohci, OHCI1394_PhyControl);

	return (r & 0x00ff0000) >> 16;
}

/* Writes to a PHY register of an OHCI-1394 controller */
static inline void __init set_phy_reg(struct ti_ohci *ohci, u8 addr, u8 data)
{
	int i;

	reg_write(ohci, OHCI1394_PhyControl, (addr << 8) | data | 0x00004000);

	for (i = 0; i < OHCI_LOOP_COUNT; i++) {
		u32 r = reg_read(ohci, OHCI1394_PhyControl);
		if (!(r & 0x00004000))
			break;
		mdelay(1);
	}
}

/* Resets an OHCI-1394 controller (for sane state before initialization) */
static inline void __init init_ohci1394_soft_reset(struct ti_ohci *ohci) {
	int i;

	reg_write(ohci, OHCI1394_HCControlSet, OHCI1394_HCControl_softReset);

	for (i = 0; i < OHCI_LOOP_COUNT; i++) {
		if (!(reg_read(ohci, OHCI1394_HCControlSet)
				   & OHCI1394_HCControl_softReset))
			break;
		mdelay(1);
	}
}

/* Basic OHCI-1394 register and port inititalization */
static inline void __init init_ohci1394_initialize(struct ti_ohci *ohci)
{
	quadlet_t bus_options;
	int num_ports, i;

	/* Put some defaults to these undefined bus options */
	bus_options = reg_read(ohci, OHCI1394_BusOptions);
	bus_options |=  0x60000000; /* Enable CMC and ISC */
	bus_options &= ~0x00ff0000; /* XXX: Set cyc_clk_acc to zero for now */
	bus_options &= ~0x18000000; /* Disable PMC and BMC */
	reg_write(ohci, OHCI1394_BusOptions, bus_options);

	/* Set the bus number */
	reg_write(ohci, OHCI1394_NodeID, 0x0000ffc0);

	/* Enable posted writes */
	reg_write(ohci, OHCI1394_HCControlSet,
			OHCI1394_HCControl_postedWriteEnable);

	/* Clear link control register */
	reg_write(ohci, OHCI1394_LinkControlClear, 0xffffffff);

	/* enable phys */
	reg_write(ohci, OHCI1394_LinkControlSet,
			OHCI1394_LinkControl_RcvPhyPkt);

	/* Don't accept phy packets into AR request context */
	reg_write(ohci, OHCI1394_LinkControlClear, 0x00000400);

	/* Clear the Isochonouys interrupt masks */
	reg_write(ohci, OHCI1394_IsoRecvIntMaskClear, 0xffffffff);
	reg_write(ohci, OHCI1394_IsoRecvIntEventClear, 0xffffffff);
	reg_write(ohci, OHCI1394_IsoXmitIntMaskClear, 0xffffffff);
	reg_write(ohci, OHCI1394_IsoXmitIntEventClear, 0xffffffff);

	/* Accept asyncronous transfer requests from all nodes for now */
	reg_write(ohci,OHCI1394_AsReqFilterHiSet, 0x80000000);

	/* Specify asyncronous transfer retries */
	reg_write(ohci, OHCI1394_ATRetries,
		  OHCI1394_MAX_AT_REQ_RETRIES |
		  (OHCI1394_MAX_AT_RESP_RETRIES<<4) |
		  (OHCI1394_MAX_PHYS_RESP_RETRIES<<8));

	/* We don't want hardware swapping */
	reg_write(ohci, OHCI1394_HCControlClear, OHCI1394_HCControl_noByteSwap);

	/* Enable link */
	reg_write(ohci, OHCI1394_HCControlSet, OHCI1394_HCControl_linkEnable);

	/* If anything is connected to a port, make sure it is enabled */
	num_ports = get_phy_reg(ohci, 2) & 0xf;
	for (i = 0; i < num_ports; i++) {
		unsigned int status;

		set_phy_reg(ohci, 7, i);
		status = get_phy_reg(ohci, 8);

		if (status & 0x20)
			set_phy_reg(ohci, 8, status & ~1);
	}
}

static inline void __init init_ohci1394_wait_for_busresets(struct ti_ohci *ohci)
{
	int i, events;

	for (i=0; i < 9; i++) {
		mdelay(200);
		events = reg_read(ohci, OHCI1394_IntEventSet);
		if (events & OHCI1394_busReset)
			reg_write(ohci, OHCI1394_IntEventClear,
					OHCI1394_busReset);
	}
}

static inline void __init init_ohci1394_enable_physical_dma(struct ti_ohci *hci)
{
	reg_write(hci, OHCI1394_PhyReqFilterHiSet, 0xffffffff);
	reg_write(hci, OHCI1394_PhyReqFilterLoSet, 0xffffffff);
	reg_write(hci, OHCI1394_PhyUpperBound, 0xffff0000);
}

static inline void __init init_ohci1394_reset_and_init_dma(struct ti_ohci *ohci)
{
	/* Start off with a soft reset, clears everything to a sane state. */
	init_ohci1394_soft_reset(ohci);

	/* Accessing some registers without LPS enabled may cause lock up */
	reg_write(ohci, OHCI1394_HCControlSet, OHCI1394_HCControl_LPS);

	/* Disable and clear interrupts */
	reg_write(ohci, OHCI1394_IntEventClear, 0xffffffff);
	reg_write(ohci, OHCI1394_IntMaskClear, 0xffffffff);

	mdelay(50); /* Wait 50msec to make sure we have full link enabled */

	init_ohci1394_initialize(ohci);
	/*
	 * The initialization causes at least one IEEE1394 bus reset. Enabling
	 * physical DMA only works *after* *all* bus resets have calmed down:
	 */
	init_ohci1394_wait_for_busresets(ohci);

	/* We had to wait and do this now if we want to debug early problems */
	init_ohci1394_enable_physical_dma(ohci);
}

static inline void __init init_ohci1394_controller(int num, int slot, int func)
{
	unsigned long ohci_base;
	struct ti_ohci ohci;

	printk(KERN_INFO "init_ohci1394_dma: initializing OHCI-1394"
			 " at %02x:%02x.%x\n", num, slot, func);

	ohci_base = read_pci_config(num, slot, func, PCI_BASE_ADDRESS_0+(0<<2))
						   & PCI_BASE_ADDRESS_MEM_MASK;

	set_fixmap_nocache(FIX_OHCI1394_BASE, ohci_base);

	ohci.registers = (void *)fix_to_virt(FIX_OHCI1394_BASE);

	init_ohci1394_reset_and_init_dma(&ohci);
}

void __init init_ohci1394_dma_on_all_controllers(void)
{
	int num, slot, func;

	if (!early_pci_allowed())
		return;

	/* Poor man's PCI discovery, the only thing we can do at early boot */
	for (num = 0; num < 32; num++) {
		for (slot = 0; slot < 32; slot++) {
			for (func = 0; func < 8; func++) {
				u32 class = read_pci_config(num,slot,func,
							PCI_CLASS_REVISION);
				if ((class == 0xffffffff))
					continue; /* No device at this func */

				if (class>>8 != PCI_CLASS_SERIAL_FIREWIRE_OHCI)
					continue; /* Not an OHCI-1394 device */

				init_ohci1394_controller(num, slot, func);
				break; /* Assume one controller per device */
			}
		}
	}
	printk(KERN_INFO "init_ohci1394_dma: finished initializing OHCI DMA\n");
}

static int __init setup_ohci1394_dma(char *opt)
{
	if (!strcmp(opt, "early"))
		init_ohci1394_dma_early = 1;
	return 0;
}

/* passing ohci1394_dma=early on boot causes early OHCI1394 DMA initialization */
early_param("ohci1394_dma", setup_ohci1394_dma);
