

#include <linux/module.h>
#include <linux/io.h>

#include <asm/tlb.h>
#include <asm/mach/map.h>

#include <mach/common.h>

void __iomem *davinci_ioremap(unsigned long p, size_t size, unsigned int type)
{
	struct map_desc *desc = davinci_soc_info.io_desc;
	int desc_num = davinci_soc_info.io_desc_num;
	int i;

	for (i = 0; i < desc_num; i++, desc++) {
		unsigned long iophys = __pfn_to_phys(desc->pfn);
		unsigned long iosize = desc->length;

		if (p >= iophys && (p + size) <= (iophys + iosize))
			return __io(desc->virtual + p - iophys);
	}

	return __arm_ioremap_caller(p, size, type,
					__builtin_return_address(0));
}
EXPORT_SYMBOL(davinci_ioremap);

void davinci_iounmap(volatile void __iomem *addr)
{
	unsigned long virt = (unsigned long)addr;

	if (virt >= VMALLOC_START && virt < VMALLOC_END)
		__iounmap(addr);
}
EXPORT_SYMBOL(davinci_iounmap);
