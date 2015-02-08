

#include "drmP.h"
#include "drm.h"
#include "mga_drm.h"
#include "mga_drv.h"

#include "drm_pciids.h"

static int mga_driver_device_is_agp(struct drm_device * dev);

static struct pci_device_id pciidlist[] = {
	mga_PCI_IDS
};

static struct drm_driver driver = {
	.driver_features =
	    DRIVER_USE_AGP | DRIVER_USE_MTRR | DRIVER_PCI_DMA |
	    DRIVER_HAVE_DMA | DRIVER_HAVE_IRQ | DRIVER_IRQ_SHARED,
	.dev_priv_size = sizeof(drm_mga_buf_priv_t),
	.load = mga_driver_load,
	.unload = mga_driver_unload,
	.lastclose = mga_driver_lastclose,
	.dma_quiescent = mga_driver_dma_quiescent,
	.device_is_agp = mga_driver_device_is_agp,
	.get_vblank_counter = mga_get_vblank_counter,
	.enable_vblank = mga_enable_vblank,
	.disable_vblank = mga_disable_vblank,
	.irq_preinstall = mga_driver_irq_preinstall,
	.irq_postinstall = mga_driver_irq_postinstall,
	.irq_uninstall = mga_driver_irq_uninstall,
	.irq_handler = mga_driver_irq_handler,
	.reclaim_buffers = drm_core_reclaim_buffers,
	.get_map_ofs = drm_core_get_map_ofs,
	.get_reg_ofs = drm_core_get_reg_ofs,
	.ioctls = mga_ioctls,
	.dma_ioctl = mga_dma_buffers,
	.fops = {
		.owner = THIS_MODULE,
		.open = drm_open,
		.release = drm_release,
		.unlocked_ioctl = drm_ioctl,
		.mmap = drm_mmap,
		.poll = drm_poll,
		.fasync = drm_fasync,
#ifdef CONFIG_COMPAT
		.compat_ioctl = mga_compat_ioctl,
#endif
	},
	.pci_driver = {
		.name = DRIVER_NAME,
		.id_table = pciidlist,
	},

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static int __init mga_init(void)
{
	driver.num_ioctls = mga_max_ioctl;
	return drm_init(&driver);
}

static void __exit mga_exit(void)
{
	drm_exit(&driver);
}

module_init(mga_init);
module_exit(mga_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");

static int mga_driver_device_is_agp(struct drm_device * dev)
{
	const struct pci_dev *const pdev = dev->pdev;

	/* There are PCI versions of the G450.  These cards have the
	 * same PCI ID as the AGP G450, but have an additional PCI-to-PCI
	 * bridge chip.  We detect these cards, which are not currently
	 * supported by this driver, by looking at the device ID of the
	 * bus the "card" is on.  If vendor is 0x3388 (Hint Corp) and the
	 * device is 0x0021 (HB6 Universal PCI-PCI bridge), we reject the
	 * device.
	 */

	if ((pdev->device == 0x0525) && pdev->bus->self
	    && (pdev->bus->self->vendor == 0x3388)
	    && (pdev->bus->self->device == 0x0021)) {
		return 0;
	}

	return 2;
}
