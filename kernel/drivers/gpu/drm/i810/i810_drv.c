

#include "drmP.h"
#include "drm.h"
#include "i810_drm.h"
#include "i810_drv.h"

#include "drm_pciids.h"

static struct pci_device_id pciidlist[] = {
	i810_PCI_IDS
};

static struct drm_driver driver = {
	.driver_features =
	    DRIVER_USE_AGP | DRIVER_REQUIRE_AGP | DRIVER_USE_MTRR |
	    DRIVER_HAVE_DMA | DRIVER_DMA_QUEUE,
	.dev_priv_size = sizeof(drm_i810_buf_priv_t),
	.load = i810_driver_load,
	.lastclose = i810_driver_lastclose,
	.preclose = i810_driver_preclose,
	.device_is_agp = i810_driver_device_is_agp,
	.reclaim_buffers_locked = i810_driver_reclaim_buffers_locked,
	.dma_quiescent = i810_driver_dma_quiescent,
	.get_map_ofs = drm_core_get_map_ofs,
	.get_reg_ofs = drm_core_get_reg_ofs,
	.ioctls = i810_ioctls,
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

static int __init i810_init(void)
{
	driver.num_ioctls = i810_max_ioctl;
	return drm_init(&driver);
}

static void __exit i810_exit(void)
{
	drm_exit(&driver);
}

module_init(i810_init);
module_exit(i810_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");
