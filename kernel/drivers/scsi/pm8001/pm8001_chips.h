

#ifndef _PM8001_CHIPS_H_
#define _PM8001_CHIPS_H_

static inline u32 pm8001_read_32(void *virt_addr)
{
	return *((u32 *)virt_addr);
}

static inline void pm8001_write_32(void *addr, u32 offset, u32 val)
{
	*((u32 *)(addr + offset)) = val;
}

static inline u32 pm8001_cr32(struct pm8001_hba_info *pm8001_ha, u32 bar,
		u32 offset)
{
	return readl(pm8001_ha->io_mem[bar].memvirtaddr + offset);
}

static inline void pm8001_cw32(struct pm8001_hba_info *pm8001_ha, u32 bar,
		u32 addr, u32 val)
{
	writel(val, pm8001_ha->io_mem[bar].memvirtaddr + addr);
}
static inline u32 pm8001_mr32(void __iomem *addr, u32 offset)
{
	return readl(addr + offset);
}
static inline void pm8001_mw32(void __iomem *addr, u32 offset, u32 val)
{
	writel(val, addr + offset);
}
static inline u32 get_pci_bar_index(u32 pcibar)
{
		switch (pcibar) {
		case 0x18:
		case 0x1C:
			return 1;
		case 0x20:
			return 2;
		case 0x24:
			return 3;
		default:
			return 0;
	}
}

#endif  /* _PM8001_CHIPS_H_ */

