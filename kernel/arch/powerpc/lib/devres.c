

#include <linux/device.h>	/* devres_*(), devm_ioremap_release() */
#include <linux/gfp.h>
#include <linux/io.h>		/* ioremap_flags() */
#include <linux/module.h>	/* EXPORT_SYMBOL() */

void __iomem *devm_ioremap_prot(struct device *dev, resource_size_t offset,
				 size_t size, unsigned long flags)
{
	void __iomem **ptr, *addr;

	ptr = devres_alloc(devm_ioremap_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return NULL;

	addr = ioremap_flags(offset, size, flags);
	if (addr) {
		*ptr = addr;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	return addr;
}
EXPORT_SYMBOL(devm_ioremap_prot);
