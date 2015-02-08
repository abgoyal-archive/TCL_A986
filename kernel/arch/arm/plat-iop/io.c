
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <mach/hardware.h>

void * __iomem __iop3xx_ioremap(unsigned long cookie, size_t size,
	unsigned int mtype)
{
	void __iomem * retval;

	switch (cookie) {
	case IOP3XX_PCI_LOWER_IO_PA ... IOP3XX_PCI_UPPER_IO_PA:
		retval = (void *) IOP3XX_PCI_IO_PHYS_TO_VIRT(cookie);
		break;
	case IOP3XX_PERIPHERAL_PHYS_BASE ... IOP3XX_PERIPHERAL_UPPER_PA:
		retval = (void *) IOP3XX_PMMR_PHYS_TO_VIRT(cookie);
		break;
	default:
		retval = __arm_ioremap_caller(cookie, size, mtype,
				__builtin_return_address(0));
	}

	return retval;
}
EXPORT_SYMBOL(__iop3xx_ioremap);

void __iop3xx_iounmap(void __iomem *addr)
{
	extern void __iounmap(volatile void __iomem *addr);

	switch ((u32) addr) {
	case IOP3XX_PCI_LOWER_IO_VA ... IOP3XX_PCI_UPPER_IO_VA:
	case IOP3XX_PERIPHERAL_VIRT_BASE ... IOP3XX_PERIPHERAL_UPPER_VA:
		goto skip;
	}
	__iounmap(addr);

skip:
	return;
}
EXPORT_SYMBOL(__iop3xx_iounmap);
