

#include <linux/pm.h>
#include <linux/pm_runtime.h>

#ifdef CONFIG_PM_RUNTIME
int pm_generic_runtime_idle(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm && pm->runtime_idle) {
		int ret = pm->runtime_idle(dev);
		if (ret)
			return ret;
	}

	pm_runtime_suspend(dev);
	return 0;
}
EXPORT_SYMBOL_GPL(pm_generic_runtime_idle);

int pm_generic_runtime_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;
	int ret;

	ret = pm && pm->runtime_suspend ? pm->runtime_suspend(dev) : -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(pm_generic_runtime_suspend);

int pm_generic_runtime_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;
	int ret;

	ret = pm && pm->runtime_resume ? pm->runtime_resume(dev) : -EINVAL;

	return ret;
}
EXPORT_SYMBOL_GPL(pm_generic_runtime_resume);
#endif /* CONFIG_PM_RUNTIME */

#ifdef CONFIG_PM_SLEEP
static int __pm_generic_call(struct device *dev, int event)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;
	int (*callback)(struct device *);

	if (!pm || pm_runtime_suspended(dev))
		return 0;

	switch (event) {
	case PM_EVENT_SUSPEND:
		callback = pm->suspend;
		break;
	case PM_EVENT_FREEZE:
		callback = pm->freeze;
		break;
	case PM_EVENT_HIBERNATE:
		callback = pm->poweroff;
		break;
	case PM_EVENT_THAW:
		callback = pm->thaw;
		break;
	default:
		callback = NULL;
		break;
	}

	return callback ? callback(dev) : 0;
}

int pm_generic_suspend(struct device *dev)
{
	return __pm_generic_call(dev, PM_EVENT_SUSPEND);
}
EXPORT_SYMBOL_GPL(pm_generic_suspend);

int pm_generic_freeze(struct device *dev)
{
	return __pm_generic_call(dev, PM_EVENT_FREEZE);
}
EXPORT_SYMBOL_GPL(pm_generic_freeze);

int pm_generic_poweroff(struct device *dev)
{
	return __pm_generic_call(dev, PM_EVENT_HIBERNATE);
}
EXPORT_SYMBOL_GPL(pm_generic_poweroff);

int pm_generic_thaw(struct device *dev)
{
	return __pm_generic_call(dev, PM_EVENT_THAW);
}
EXPORT_SYMBOL_GPL(pm_generic_thaw);

static int __pm_generic_resume(struct device *dev, int event)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;
	int (*callback)(struct device *);
	int ret;

	if (!pm)
		return 0;

	switch (event) {
	case PM_EVENT_RESUME:
		callback = pm->resume;
		break;
	case PM_EVENT_RESTORE:
		callback = pm->restore;
		break;
	default:
		callback = NULL;
		break;
	}

	if (!callback)
		return 0;

	ret = callback(dev);
	if (!ret) {
		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
	}

	return ret;
}

int pm_generic_resume(struct device *dev)
{
	return __pm_generic_resume(dev, PM_EVENT_RESUME);
}
EXPORT_SYMBOL_GPL(pm_generic_resume);

int pm_generic_restore(struct device *dev)
{
	return __pm_generic_resume(dev, PM_EVENT_RESTORE);
}
EXPORT_SYMBOL_GPL(pm_generic_restore);
#endif /* CONFIG_PM_SLEEP */

struct dev_pm_ops generic_subsys_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend = pm_generic_suspend,
	.resume = pm_generic_resume,
	.freeze = pm_generic_freeze,
	.thaw = pm_generic_thaw,
	.poweroff = pm_generic_poweroff,
	.restore = pm_generic_restore,
#endif
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = pm_generic_runtime_suspend,
	.runtime_resume = pm_generic_runtime_resume,
	.runtime_idle = pm_generic_runtime_idle,
#endif
};
EXPORT_SYMBOL_GPL(generic_subsys_pm_ops);
