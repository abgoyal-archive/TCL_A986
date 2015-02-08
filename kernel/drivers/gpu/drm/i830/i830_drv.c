

#include "drmP.h"
#include "drm.h"
#include "i830_drm.h"
#include "i830_drv.h"

#include "drm_pciids.h"

static struct pci_device_id pciidlist[] = {
	i830_PCI_IDS
};

static struct drm_driver driver = {
	.driver_features =
	    DRIVER_USE_AGP | DRIVER_REQUIRE_AGP | DRIVER_USE_MTRR |
	    DRIVER_HAVE_DMA | DRIVER_DMA_QUEUE,
#if USE_IRQS
	.driver_features |= DRIVER_HAVE_IRQ | DRIVER_SHARED_IRQ,
#endif
	.dev_priv_size = sizeof(drm_i830_buf_priv_t),
	.load = i830_driver_load,
	.lastclose = i830_driver_lastclose,
	.preclose = i830_driver_preclose,
	.device_is_agp = i830_driver_device_is_agp,
	.reclaim_buffers_locked = i830_driver_reclaim_buffers_locked,
	.dma_quiescent = i830_driver_dma_quiescent,
	.get_map_ofs = drm_core_get_map_ofs,
	.get_reg_ofs = drm_core_get_reg_ofs,
#if USE_IRQS
	.irq_preinstall = i830_driver_irq_preinstall,
	.irq_postinstall = i830_driver_irq_postinstall,
	.irq_uninstall = i830_driver_irq_uninstall,
	.irq_handler = i830_driver_irq_handler,
#endif
	.ioctls = i830_ioctls,
	.fops = {
		 .owner = THIS_MODULE,
		 .open = drm_open,
		 .release = drm_release,
		 .unlocked_ioctl = drm_ioctl,
		 .mmap = drm_mmap,
		 .poll = drm_poll,
		 .fasync = drm_fasync,
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

static int __init i830_init(void)
{
	driver.num_ioctls = i830_max_ioctl;
	return drm_init(&driver);
}

static void __exit i830_exit(void)
{
	drm_exit(&driver);
}

module_init(i830_init);
module_exit(i830_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");
