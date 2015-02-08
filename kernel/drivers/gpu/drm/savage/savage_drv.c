

#include "drmP.h"
#include "savage_drm.h"
#include "savage_drv.h"

#include "drm_pciids.h"

static struct pci_device_id pciidlist[] = {
	savage_PCI_IDS
};

static struct drm_driver driver = {
	.driver_features =
	    DRIVER_USE_AGP | DRIVER_USE_MTRR | DRIVER_HAVE_DMA | DRIVER_PCI_DMA,
	.dev_priv_size = sizeof(drm_savage_buf_priv_t),
	.load = savage_driver_load,
	.firstopen = savage_driver_firstopen,
	.lastclose = savage_driver_lastclose,
	.unload = savage_driver_unload,
	.reclaim_buffers = savage_reclaim_buffers,
	.get_map_ofs = drm_core_get_map_ofs,
	.get_reg_ofs = drm_core_get_reg_ofs,
	.ioctls = savage_ioctls,
	.dma_ioctl = savage_bci_buffers,
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

static int __init savage_init(void)
{
	driver.num_ioctls = savage_max_ioctl;
	return drm_init(&driver);
}

static void __exit savage_exit(void)
{
	drm_exit(&driver);
}

module_init(savage_init);
module_exit(savage_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");
