

#include "drmP.h"
#include "tdfx_drv.h"

#include "drm_pciids.h"

static struct pci_device_id pciidlist[] = {
	tdfx_PCI_IDS
};

static struct drm_driver driver = {
	.driver_features = DRIVER_USE_MTRR,
	.reclaim_buffers = drm_core_reclaim_buffers,
	.get_map_ofs = drm_core_get_map_ofs,
	.get_reg_ofs = drm_core_get_reg_ofs,
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

static int __init tdfx_init(void)
{
	return drm_init(&driver);
}

static void __exit tdfx_exit(void)
{
	drm_exit(&driver);
}

module_init(tdfx_init);
module_exit(tdfx_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");
