

#include <linux/device.h>
#include <linux/string.h>
#include <linux/pm_runtime.h>
#include <asm/atomic.h>
#include "power.h"


static const char enabled[] = "enabled";
static const char disabled[] = "disabled";

#ifdef CONFIG_PM_RUNTIME
static const char ctrl_auto[] = "auto";
static const char ctrl_on[] = "on";

static ssize_t control_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%s\n",
				dev->power.runtime_auto ? ctrl_auto : ctrl_on);
}

static ssize_t control_store(struct device * dev, struct device_attribute *attr,
			     const char * buf, size_t n)
{
	char *cp;
	int len = n;

	cp = memchr(buf, '\n', n);
	if (cp)
		len = cp - buf;
	if (len == sizeof ctrl_auto - 1 && strncmp(buf, ctrl_auto, len) == 0)
		pm_runtime_allow(dev);
	else if (len == sizeof ctrl_on - 1 && strncmp(buf, ctrl_on, len) == 0)
		pm_runtime_forbid(dev);
	else
		return -EINVAL;
	return n;
}

static DEVICE_ATTR(control, 0644, control_show, control_store);
#endif

static ssize_t
wake_show(struct device * dev, struct device_attribute *attr, char * buf)
{
	return sprintf(buf, "%s\n", device_can_wakeup(dev)
		? (device_may_wakeup(dev) ? enabled : disabled)
		: "");
}

static ssize_t
wake_store(struct device * dev, struct device_attribute *attr,
	const char * buf, size_t n)
{
	char *cp;
	int len = n;

	if (!device_can_wakeup(dev))
		return -EINVAL;

	cp = memchr(buf, '\n', n);
	if (cp)
		len = cp - buf;
	if (len == sizeof enabled - 1
			&& strncmp(buf, enabled, sizeof enabled - 1) == 0)
		device_set_wakeup_enable(dev, 1);
	else if (len == sizeof disabled - 1
			&& strncmp(buf, disabled, sizeof disabled - 1) == 0)
		device_set_wakeup_enable(dev, 0);
	else
		return -EINVAL;
	return n;
}

static DEVICE_ATTR(wakeup, 0644, wake_show, wake_store);

#ifdef CONFIG_PM_ADVANCED_DEBUG
#ifdef CONFIG_PM_RUNTIME

static ssize_t rtpm_usagecount_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&dev->power.usage_count));
}

static ssize_t rtpm_children_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", dev->power.ignore_children ?
		0 : atomic_read(&dev->power.child_count));
}

static ssize_t rtpm_enabled_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	if ((dev->power.disable_depth) && (dev->power.runtime_auto == false))
		return sprintf(buf, "disabled & forbidden\n");
	else if (dev->power.disable_depth)
		return sprintf(buf, "disabled\n");
	else if (dev->power.runtime_auto == false)
		return sprintf(buf, "forbidden\n");
	return sprintf(buf, "enabled\n");
}

static ssize_t rtpm_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev->power.runtime_error)
		return sprintf(buf, "error\n");
	switch (dev->power.runtime_status) {
	case RPM_SUSPENDED:
		return sprintf(buf, "suspended\n");
	case RPM_SUSPENDING:
		return sprintf(buf, "suspending\n");
	case RPM_RESUMING:
		return sprintf(buf, "resuming\n");
	case RPM_ACTIVE:
		return sprintf(buf, "active\n");
	}
	return -EIO;
}

static DEVICE_ATTR(runtime_usage, 0444, rtpm_usagecount_show, NULL);
static DEVICE_ATTR(runtime_active_kids, 0444, rtpm_children_show, NULL);
static DEVICE_ATTR(runtime_status, 0444, rtpm_status_show, NULL);
static DEVICE_ATTR(runtime_enabled, 0444, rtpm_enabled_show, NULL);

#endif

static ssize_t async_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%s\n",
			device_async_suspend_enabled(dev) ? enabled : disabled);
}

static ssize_t async_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t n)
{
	char *cp;
	int len = n;

	cp = memchr(buf, '\n', n);
	if (cp)
		len = cp - buf;
	if (len == sizeof enabled - 1 && strncmp(buf, enabled, len) == 0)
		device_enable_async_suspend(dev);
	else if (len == sizeof disabled - 1 && strncmp(buf, disabled, len) == 0)
		device_disable_async_suspend(dev);
	else
		return -EINVAL;
	return n;
}

static DEVICE_ATTR(async, 0644, async_show, async_store);
#endif /* CONFIG_PM_ADVANCED_DEBUG */

static struct attribute * power_attrs[] = {
#ifdef CONFIG_PM_RUNTIME
	&dev_attr_control.attr,
#endif
	&dev_attr_wakeup.attr,
#ifdef CONFIG_PM_ADVANCED_DEBUG
	&dev_attr_async.attr,
#ifdef CONFIG_PM_RUNTIME
	&dev_attr_runtime_usage.attr,
	&dev_attr_runtime_active_kids.attr,
	&dev_attr_runtime_status.attr,
	&dev_attr_runtime_enabled.attr,
#endif
#endif
	NULL,
};
static struct attribute_group pm_attr_group = {
	.name	= "power",
	.attrs	= power_attrs,
};

int dpm_sysfs_add(struct device * dev)
{
	return sysfs_create_group(&dev->kobj, &pm_attr_group);
}

void dpm_sysfs_remove(struct device * dev)
{
	sysfs_remove_group(&dev->kobj, &pm_attr_group);
}
