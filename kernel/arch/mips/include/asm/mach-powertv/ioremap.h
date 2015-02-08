
#ifndef __ASM_MACH_POWERTV_IOREMAP_H
#define __ASM_MACH_POWERTV_IOREMAP_H

#include <linux/types.h>

#define LOW_MEM_BOUNDARY_PHYS	0x20000000
#define LOW_MEM_BOUNDARY_MASK	(~(LOW_MEM_BOUNDARY_PHYS - 1))

extern unsigned long phys_to_bus_offset;

#ifdef CONFIG_HIGHMEM
#define MEM_GAP_PHYS		0x60000000
#define phys_to_bus(x) (((x) < LOW_MEM_BOUNDARY_PHYS) ? \
	((x) + phys_to_bus_offset) : (x))
#define bus_to_phys(x) (((x) < MEM_GAP_PHYS_ADDR) ? \
	((x) - phys_to_bus_offset) : (x))
#else
#define phys_to_bus(x) ((x) + phys_to_bus_offset)
#define bus_to_phys(x) ((x) - phys_to_bus_offset)
#endif

static inline int asic_is_device_addr(phys_t addr)
{
	return !((phys_t)addr & (phys_t) LOW_MEM_BOUNDARY_MASK);
}

static inline int asic_is_lowmem_ram_addr(phys_t addr)
{
	/*
	 * The RAM always starts at the following address in the processor's
	 * physical address space
	 */
	static const phys_t phys_ram_base = 0x10000000;
	phys_t bus_ram_base;

	bus_ram_base = phys_to_bus_offset + phys_ram_base;

	return addr >= bus_ram_base &&
		addr < (bus_ram_base + (LOW_MEM_BOUNDARY_PHYS - phys_ram_base));
}

static inline phys_t fixup_bigphys_addr(phys_t phys_addr, phys_t size)
{
	return phys_addr;
}

static inline void __iomem *plat_ioremap(phys_t offset, unsigned long size,
	unsigned long flags)
{
	return NULL;
}

static inline int plat_iounmap(const volatile void __iomem *addr)
{
	return 0;
}
#endif /* __ASM_MACH_POWERTV_IOREMAP_H */
