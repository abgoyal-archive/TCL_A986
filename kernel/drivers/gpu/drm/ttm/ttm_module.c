
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sched.h>
#include "ttm/ttm_module.h"
#include "drm_sysfs.h"

static DECLARE_WAIT_QUEUE_HEAD(exit_q);
atomic_t device_released;

static struct device_type ttm_drm_class_type = {
	.name = "ttm",
	/**
	 * Add pm ops here.
	 */
};

static void ttm_drm_class_device_release(struct device *dev)
{
	atomic_set(&device_released, 1);
	wake_up_all(&exit_q);
}

static struct device ttm_drm_class_device = {
	.type = &ttm_drm_class_type,
	.release = &ttm_drm_class_device_release
};

struct kobject *ttm_get_kobj(void)
{
	struct kobject *kobj = &ttm_drm_class_device.kobj;
	BUG_ON(kobj == NULL);
	return kobj;
}

static int __init ttm_init(void)
{
	int ret;

	ret = dev_set_name(&ttm_drm_class_device, "ttm");
	if (unlikely(ret != 0))
		return ret;

	ttm_global_init();

	atomic_set(&device_released, 0);
	ret = drm_class_device_register(&ttm_drm_class_device);
	if (unlikely(ret != 0))
		goto out_no_dev_reg;

	return 0;
out_no_dev_reg:
	atomic_set(&device_released, 1);
	wake_up_all(&exit_q);
	ttm_global_release();
	return ret;
}

static void __exit ttm_exit(void)
{
	drm_class_device_unregister(&ttm_drm_class_device);

	/**
	 * Refuse to unload until the TTM device is released.
	 * Not sure this is 100% needed.
	 */

	wait_event(exit_q, atomic_read(&device_released) == 1);
	ttm_global_release();
}

module_init(ttm_init);
module_exit(ttm_exit);

MODULE_AUTHOR("Thomas Hellstrom, Jerome Glisse");
MODULE_DESCRIPTION("TTM memory manager subsystem (for DRM device)");
MODULE_LICENSE("GPL and additional rights");
