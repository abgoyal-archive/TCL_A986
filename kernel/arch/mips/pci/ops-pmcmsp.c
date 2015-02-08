

#define PCI_COUNTERS	1

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>

#if defined(CONFIG_PROC_FS) && defined(PCI_COUNTERS)
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif /* CONFIG_PROC_FS && PCI_COUNTERS */

#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/byteorder.h>
#if defined(CONFIG_PMC_MSP7120_GW) || defined(CONFIG_PMC_MSP7120_EVAL)
#include <asm/mipsmtregs.h>
#endif

#include <msp_prom.h>
#include <msp_cic_int.h>
#include <msp_pci.h>
#include <msp_regs.h>
#include <msp_regops.h>

#define PCI_ACCESS_READ		0
#define PCI_ACCESS_WRITE	1

#if defined(CONFIG_PROC_FS) && defined(PCI_COUNTERS)
static char proc_init;
extern struct proc_dir_entry *proc_bus_pci_dir;
unsigned int pci_int_count[32];

static void pci_proc_init(void);

static int read_msp_pci_counts(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int i;
	int len = 0;
	unsigned int intcount, total = 0;

	for (i = 0; i < 32; ++i) {
		intcount = pci_int_count[i];
		if (intcount != 0) {
			len += sprintf(page + len, "[%d] = %u\n", i, intcount);
			total += intcount;
		}
	}

	len += sprintf(page + len, "total = %u\n", total);
	if (len <= off+count)
		*eof = 1;

	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;
}

static int gen_pci_cfg_wr(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	unsigned char where = 0; /* Write to static Device/Vendor ID */
	unsigned char bus_num = 0; /* Bus 0 */
	unsigned char dev_fn = 0xF; /* Arbitrary device number */
	u32 wr_data = 0xFF00AA00; /* Arbitrary data */
	struct msp_pci_regs *preg = (void *)PCI_BASE_REG;
	int len = 0;
	unsigned long value;
	int intr;

	len += sprintf(page + len, "PMC MSP PCI: Beginning\n");

	if (proc_init == 0) {
		pci_proc_init();
		proc_init = ~0;
	}

	len += sprintf(page + len, "PMC MSP PCI: Before Cfg Wr\n");

	/*
	 * Generate PCI Configuration Write Cycle
	 */

	/* Clear cause register bits */
	preg->if_status = ~(BPCI_IFSTATUS_BC0F | BPCI_IFSTATUS_BC1F);

	/* Setup address that is to appear on PCI bus */
	preg->config_addr = BPCI_CFGADDR_ENABLE |
		(bus_num << BPCI_CFGADDR_BUSNUM_SHF) |
		(dev_fn << BPCI_CFGADDR_FUNCTNUM_SHF) |
		(where & 0xFC);

	value = cpu_to_le32(wr_data);

	/* Launch the PCI configuration write cycle */
	*PCI_CONFIG_SPACE_REG = value;

	/*
	 * Check if the PCI configuration cycle (rd or wr) succeeded, by
	 * checking the status bits for errors like master or target abort.
	 */
	intr = preg->if_status;

	len += sprintf(page + len, "PMC MSP PCI: After Cfg Wr\n");

	/* Handle STDOUT calculations */
	if (len <= off+count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	return len;
}

static void pci_proc_init(void)
{
	create_proc_read_entry("pmc_msp_pci_rd_cnt", 0, NULL,
				read_msp_pci_counts, NULL);
	create_proc_read_entry("pmc_msp_pci_cfg_wr", 0, NULL,
				gen_pci_cfg_wr, NULL);
}
#endif /* CONFIG_PROC_FS && PCI_COUNTERS */

static DEFINE_SPINLOCK(bpci_lock);

static struct resource pci_io_resource = {
	.name	= "pci IO space",
	.start	= 0x04,
	.end	= 0x0FFF,
	.flags	= IORESOURCE_IO	/* I/O space */
};

static struct resource pci_mem_resource = {
	.name	= "pci memory space",
	.start	= MSP_PCI_SPACE_BASE,
	.end	= MSP_PCI_SPACE_END,
	.flags	= IORESOURCE_MEM	 /* memory space */
};

static int bpci_interrupt(int irq, void *dev_id)
{
	struct msp_pci_regs *preg = (void *)PCI_BASE_REG;
	unsigned int stat = preg->if_status;

#if defined(CONFIG_PROC_FS) && defined(PCI_COUNTERS)
	int i;
	for (i = 0; i < 32; ++i) {
		if ((1 << i) & stat)
			++pci_int_count[i];
	}
#endif /* PROC_FS && PCI_COUNTERS */

	/* printk("PCI ISR: Status=%08X\n", stat); */

	/* write to clear all asserted interrupts */
	preg->if_status = stat;

	return PCIBIOS_SUCCESSFUL;
}

int msp_pcibios_config_access(unsigned char access_type,
				struct pci_bus *bus,
				unsigned int devfn,
				unsigned char where,
				u32 *data)
{
	struct msp_pci_regs *preg = (void *)PCI_BASE_REG;
	unsigned char bus_num = bus->number;
	unsigned char dev_fn = (unsigned char)devfn;
	unsigned long flags;
	unsigned long intr;
	unsigned long value;
	static char pciirqflag;
	int ret;
#if defined(CONFIG_PMC_MSP7120_GW) || defined(CONFIG_PMC_MSP7120_EVAL)
	unsigned int	vpe_status;
#endif

#if defined(CONFIG_PROC_FS) && defined(PCI_COUNTERS)
	if (proc_init == 0) {
		pci_proc_init();
		proc_init = ~0;
	}
#endif /* CONFIG_PROC_FS && PCI_COUNTERS */

	/*
	 * Just the first time this function invokes, allocate
	 * an interrupt line for PCI host status interrupts. The
	 * allocation assigns an interrupt handler to the interrupt.
	 */
	if (pciirqflag == 0) {
		ret = request_irq(MSP_INT_PCI,/* Hardcoded internal MSP7120 wiring */
				bpci_interrupt,
				IRQF_SHARED | IRQF_DISABLED,
				"PMC MSP PCI Host",
				preg);
		if (ret != 0)
			return ret;
		pciirqflag = ~0;
	}

#if defined(CONFIG_PMC_MSP7120_GW) || defined(CONFIG_PMC_MSP7120_EVAL)
	local_irq_save(flags);
	vpe_status = dvpe();
#else
	spin_lock_irqsave(&bpci_lock, flags);
#endif

	/*
	 * Clear PCI cause register bits.
	 *
	 * In Polo, the PCI Host had a dedicated DMA called the
	 * Block Copy (not to be confused with the general purpose Block
	 * Copy Engine block). There appear to have been special interrupts
	 * for this Block Copy, called Block Copy 0 Fault (BC0F) and
	 * Block Copy 1 Fault (BC1F). MSP4200 and MSP7120 don't have this
	 * dedicated Block Copy block, so these two interrupts are now
	 * marked reserved. In case the  Block Copy is resurrected in a
	 * future design, maintain the code that treats these two interrupts
	 * specially.
	 *
	 * Write to clear all interrupts in the PCI status register, aside
	 * from BC0F and BC1F.
	 */
	preg->if_status = ~(BPCI_IFSTATUS_BC0F | BPCI_IFSTATUS_BC1F);

	/* Setup address that is to appear on PCI bus */
	preg->config_addr = BPCI_CFGADDR_ENABLE	|
		(bus_num << BPCI_CFGADDR_BUSNUM_SHF) |
		(dev_fn << BPCI_CFGADDR_FUNCTNUM_SHF) |
		(where & 0xFC);

	/* IF access is a PCI configuration write */
	if (access_type == PCI_ACCESS_WRITE) {
		value = cpu_to_le32(*data);
		*PCI_CONFIG_SPACE_REG = value;
	} else {
		/* ELSE access is a PCI configuration read */
		value = le32_to_cpu(*PCI_CONFIG_SPACE_REG);
		*data = value;
	}

	/*
	 * Check if the PCI configuration cycle (rd or wr) succeeded, by
	 * checking the status bits for errors like master or target abort.
	 */
	intr = preg->if_status;

	/* Clear config access */
	preg->config_addr = 0;

	/* IF error occurred */
	if (intr & ~(BPCI_IFSTATUS_BC0F | BPCI_IFSTATUS_BC1F)) {
		/* Clear status bits */
		preg->if_status = ~(BPCI_IFSTATUS_BC0F | BPCI_IFSTATUS_BC1F);

#if defined(CONFIG_PMC_MSP7120_GW) || defined(CONFIG_PMC_MSP7120_EVAL)
		evpe(vpe_status);
		local_irq_restore(flags);
#else
		spin_unlock_irqrestore(&bpci_lock, flags);
#endif

		return -1;
	}

#if defined(CONFIG_PMC_MSP7120_GW) || defined(CONFIG_PMC_MSP7120_EVAL)
	evpe(vpe_status);
	local_irq_restore(flags);
#else
	spin_unlock_irqrestore(&bpci_lock, flags);
#endif

	return PCIBIOS_SUCCESSFUL;
}

static int
msp_pcibios_read_config_byte(struct pci_bus *bus,
				unsigned int devfn,
				int where,
				u32 *val)
{
	u32 data = 0;

	/*
	 * If the config access did not complete normally (e.g., underwent
	 * master abort) do the PCI compliant thing, which is to supply an
	 * all ones value.
	 */
	if (msp_pcibios_config_access(PCI_ACCESS_READ, bus, devfn,
					where, &data)) {
		*val = 0xFFFFFFFF;
		return -1;
	}

	*val = (data >> ((where & 3) << 3)) & 0x0ff;

	return PCIBIOS_SUCCESSFUL;
}

static int
msp_pcibios_read_config_word(struct pci_bus *bus,
				unsigned int devfn,
				int where,
				u32 *val)
{
	u32 data = 0;

	/* if (where & 1) */	/* Commented out non-compliant code.
				 * Should allow word access to configuration
				 * registers, with only exception being when
				 * the word access would wrap around into
				 * the next dword.
				 */
	if ((where & 3) == 3) {
		*val = 0xFFFFFFFF;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	/*
	 * If the config access did not complete normally (e.g., underwent
	 * master abort) do the PCI compliant thing, which is to supply an
	 * all ones value.
	 */
	if (msp_pcibios_config_access(PCI_ACCESS_READ, bus, devfn,
					where, &data)) {
		*val = 0xFFFFFFFF;
		return -1;
	}

	*val = (data >> ((where & 3) << 3)) & 0x0ffff;

	return PCIBIOS_SUCCESSFUL;
}

static int
msp_pcibios_read_config_dword(struct pci_bus *bus,
				unsigned int devfn,
				int where,
				u32 *val)
{
	u32 data = 0;

	/* Address must be dword aligned. */
	if (where & 3) {
		*val = 0xFFFFFFFF;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	/*
	 * If the config access did not complete normally (e.g., underwent
	 * master abort) do the PCI compliant thing, which is to supply an
	 * all ones value.
	 */
	if (msp_pcibios_config_access(PCI_ACCESS_READ, bus, devfn,
					where, &data)) {
		*val = 0xFFFFFFFF;
		return -1;
	}

	*val = data;

	return PCIBIOS_SUCCESSFUL;
}

static int
msp_pcibios_write_config_byte(struct pci_bus *bus,
				unsigned int devfn,
				int where,
				u8 val)
{
	u32 data = 0;

	/* read config space */
	if (msp_pcibios_config_access(PCI_ACCESS_READ, bus, devfn,
					where, &data))
		return -1;

	/* modify the byte within the dword */
	data = (data & ~(0xff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));

	/* write back the full dword */
	if (msp_pcibios_config_access(PCI_ACCESS_WRITE, bus, devfn,
					where, &data))
		return -1;

	return PCIBIOS_SUCCESSFUL;
}

static int
msp_pcibios_write_config_word(struct pci_bus *bus,
				unsigned int devfn,
				int where,
				u16 val)
{
	u32 data = 0;

	/* Fixed non-compliance: if (where & 1) */
	if ((where & 3) == 3)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	/* read config space */
	if (msp_pcibios_config_access(PCI_ACCESS_READ, bus, devfn,
					where, &data))
		return -1;

	/* modify the word within the dword */
	data = (data & ~(0xffff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));

	/* write back the full dword */
	if (msp_pcibios_config_access(PCI_ACCESS_WRITE, bus, devfn,
					where, &data))
		return -1;

	return PCIBIOS_SUCCESSFUL;
}

static int
msp_pcibios_write_config_dword(struct pci_bus *bus,
				unsigned int devfn,
				int where,
				u32 val)
{
	/* check that address is dword aligned */
	if (where & 3)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	/* perform write */
	if (msp_pcibios_config_access(PCI_ACCESS_WRITE, bus, devfn,
					where, &val))
		return -1;

	return PCIBIOS_SUCCESSFUL;
}

int
msp_pcibios_read_config(struct pci_bus *bus,
			unsigned int	devfn,
			int where,
			int size,
			u32 *val)
{
	if (size == 1) {
		if (msp_pcibios_read_config_byte(bus, devfn, where, val)) {
			return -1;
		}
	} else if (size == 2) {
		if (msp_pcibios_read_config_word(bus, devfn, where, val)) {
			return -1;
		}
	} else if (size == 4) {
		if (msp_pcibios_read_config_dword(bus, devfn, where, val)) {
			return -1;
		}
	} else {
		*val = 0xFFFFFFFF;
		return -1;
	}

	return PCIBIOS_SUCCESSFUL;
}

int
msp_pcibios_write_config(struct pci_bus *bus,
			unsigned int devfn,
			int where,
			int size,
			u32 val)
{
	if (size == 1) {
		if (msp_pcibios_write_config_byte(bus, devfn,
						where, (u8)(0xFF & val))) {
			return -1;
		}
	} else if (size == 2) {
		if (msp_pcibios_write_config_word(bus, devfn,
						where, (u16)(0xFFFF & val))) {
			return -1;
		}
	} else if (size == 4) {
		if (msp_pcibios_write_config_dword(bus, devfn, where, val)) {
			return -1;
		}
	} else {
		return -1;
	}

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops msp_pci_ops = {
	.read = msp_pcibios_read_config,
	.write = msp_pcibios_write_config
};

static struct pci_controller msp_pci_controller = {
	.pci_ops	= &msp_pci_ops,
	.mem_resource	= &pci_mem_resource,
	.mem_offset	= 0,
	.io_map_base	= MSP_PCI_IOSPACE_BASE,
	.io_resource	= &pci_io_resource,
	.io_offset	= 0
};

void __init msp_pci_init(void)
{
	struct msp_pci_regs *preg = (void *)PCI_BASE_REG;
	u32 id;

	/* Extract Device ID */
	id = read_reg32(PCI_JTAG_DEVID_REG, 0xFFFF) >> 12;

	/* Check if JTAG ID identifies MSP7120 */
	if (!MSP_HAS_PCI(id)) {
		printk(KERN_WARNING "PCI: No PCI; id reads as %x\n", id);
		goto no_pci;
	}

	/*
	 * Enable flushing of the PCI-SDRAM queue upon a read
	 * of the SDRAM's Memory Configuration Register.
	 */
	*(unsigned long *)QFLUSH_REG_1 = 3;

	/* Configure PCI Host Controller. */
	preg->if_status	= ~0;		/* Clear cause register bits */
	preg->config_addr = 0;		/* Clear config access */
	preg->oatran	= MSP_PCI_OATRAN; /* PCI outbound addr translation */
	preg->if_mask	= 0xF8BF87C0;	/* Enable all PCI status interrupts */

	/* configure so inb(), outb(), and family are functional */
	set_io_port_base(MSP_PCI_IOSPACE_BASE);

	/* Tell Linux the details of the MSP7120 PCI Host Controller */
	register_pci_controller(&msp_pci_controller);

	return;

no_pci:
	/* Disable PCI channel */
	printk(KERN_WARNING "PCI: no host PCI bus detected\n");
}
