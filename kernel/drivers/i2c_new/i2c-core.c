
/* i2c-core.c - a device driver for the iic-bus interface		     */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/hardirq.h>
#include <linux/irqflags.h>
#include <linux/rwsem.h>
#include <linux/pm_runtime.h>
#include <asm/uaccess.h>

#include "i2c-core.h"


static DEFINE_MUTEX(core_lock);
static DEFINE_IDR(i2c_adapter_idr);

static struct device_type i2c_client_type;
static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver);

/* ------------------------------------------------------------------------- */

static const struct i2c_device_id *i2c_match_id(const struct i2c_device_id *id,
						const struct i2c_client *client)
{
	while (id->name[0]) {
		if (strcmp(client->name, id->name) == 0)
			return id;
		id++;
	}
	return NULL;
}

static int i2c_device_match(struct device *dev, struct device_driver *drv)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;

	if (!client)
		return 0;

	driver = to_i2c_driver(drv);
	/* match on an id table if there is one */
	if (driver->id_table)
		return i2c_match_id(driver->id_table, client) != NULL;

	return 0;
}

#ifdef	CONFIG_HOTPLUG

/* uevent helps with hotplug: modprobe -q $(MODALIAS) */
static int i2c_device_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct i2c_client	*client = to_i2c_client(dev);

	if (add_uevent_var(env, "MODALIAS=%s%s",
			   I2C_MODULE_PREFIX, client->name))
		return -ENOMEM;
	dev_dbg(dev, "uevent\n");
	return 0;
}

#else
#define i2c_device_uevent	NULL
#endif	/* CONFIG_HOTPLUG */

static int i2c_device_probe(struct device *dev)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;
	int status;

	if (!client)
		return 0;

	driver = to_i2c_driver(dev->driver);
	if (!driver->probe || !driver->id_table)
		return -ENODEV;
	client->driver = driver;
	if (!device_can_wakeup(&client->dev))
		device_init_wakeup(&client->dev,
					client->flags & I2C_CLIENT_WAKE);
	dev_dbg(dev, "probe\n");

	status = driver->probe(client, i2c_match_id(driver->id_table, client));
	if (status) {
		client->driver = NULL;
		i2c_set_clientdata(client, NULL);
	}
	return status;
}

static int i2c_device_remove(struct device *dev)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;
	int			status;

	if (!client || !dev->driver)
		return 0;

	driver = to_i2c_driver(dev->driver);
	if (driver->remove) {
		dev_dbg(dev, "remove\n");
		status = driver->remove(client);
	} else {
		dev->driver = NULL;
		status = 0;
	}
	if (status == 0) {
		client->driver = NULL;
		i2c_set_clientdata(client, NULL);
	}
	return status;
}

static void i2c_device_shutdown(struct device *dev)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct i2c_driver *driver;

	if (!client || !dev->driver)
		return;
	driver = to_i2c_driver(dev->driver);
	if (driver->shutdown)
		driver->shutdown(client);
}

#ifdef CONFIG_PM_SLEEP
static int i2c_legacy_suspend(struct device *dev, pm_message_t mesg)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct i2c_driver *driver;

	if (!client || !dev->driver)
		return 0;
	driver = to_i2c_driver(dev->driver);
	if (!driver->suspend)
		return 0;
	return driver->suspend(client, mesg);
}

static int i2c_legacy_resume(struct device *dev)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct i2c_driver *driver;

	if (!client || !dev->driver)
		return 0;
	driver = to_i2c_driver(dev->driver);
	if (!driver->resume)
		return 0;
	return driver->resume(client);
}

static int i2c_device_pm_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm_runtime_suspended(dev))
		return 0;

	if (pm)
		return pm->suspend ? pm->suspend(dev) : 0;

	return i2c_legacy_suspend(dev, PMSG_SUSPEND);
}

static int i2c_device_pm_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;
	int ret;

	if (pm)
		ret = pm->resume ? pm->resume(dev) : 0;
	else
		ret = i2c_legacy_resume(dev);

	if (!ret) {
		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
	}

	return ret;
}

static int i2c_device_pm_freeze(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm_runtime_suspended(dev))
		return 0;

	if (pm)
		return pm->freeze ? pm->freeze(dev) : 0;

	return i2c_legacy_suspend(dev, PMSG_FREEZE);
}

static int i2c_device_pm_thaw(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm_runtime_suspended(dev))
		return 0;

	if (pm)
		return pm->thaw ? pm->thaw(dev) : 0;

	return i2c_legacy_resume(dev);
}

static int i2c_device_pm_poweroff(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm_runtime_suspended(dev))
		return 0;

	if (pm)
		return pm->poweroff ? pm->poweroff(dev) : 0;

	return i2c_legacy_suspend(dev, PMSG_HIBERNATE);
}

static int i2c_device_pm_restore(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;
	int ret;

	if (pm)
		ret = pm->restore ? pm->restore(dev) : 0;
	else
		ret = i2c_legacy_resume(dev);

	if (!ret) {
		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
	}

	return ret;
}
#else /* !CONFIG_PM_SLEEP */
#define i2c_device_pm_suspend	NULL
#define i2c_device_pm_resume	NULL
#define i2c_device_pm_freeze	NULL
#define i2c_device_pm_thaw	NULL
#define i2c_device_pm_poweroff	NULL
#define i2c_device_pm_restore	NULL
#endif /* !CONFIG_PM_SLEEP */

static void i2c_client_dev_release(struct device *dev)
{
	kfree(to_i2c_client(dev));
}

static ssize_t
show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", dev->type == &i2c_client_type ?
		       to_i2c_client(dev)->name : to_i2c_adapter(dev)->name);
}

static ssize_t
show_modalias(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%s%s\n", I2C_MODULE_PREFIX, client->name);
}

static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);
static DEVICE_ATTR(modalias, S_IRUGO, show_modalias, NULL);

static struct attribute *i2c_dev_attrs[] = {
	&dev_attr_name.attr,
	/* modalias helps coldplug:  modprobe $(cat .../modalias) */
	&dev_attr_modalias.attr,
	NULL
};

static struct attribute_group i2c_dev_attr_group = {
	.attrs		= i2c_dev_attrs,
};

static const struct attribute_group *i2c_dev_attr_groups[] = {
	&i2c_dev_attr_group,
	NULL
};

static const struct dev_pm_ops i2c_device_pm_ops = {
	.suspend = i2c_device_pm_suspend,
	.resume = i2c_device_pm_resume,
	.freeze = i2c_device_pm_freeze,
	.thaw = i2c_device_pm_thaw,
	.poweroff = i2c_device_pm_poweroff,
	.restore = i2c_device_pm_restore,
	SET_RUNTIME_PM_OPS(
		pm_generic_runtime_suspend,
		pm_generic_runtime_resume,
		pm_generic_runtime_idle
	)
};

struct bus_type i2c_bus_type = {
	.name		= "i2c",
	.match		= i2c_device_match,
	.probe		= i2c_device_probe,
	.remove		= i2c_device_remove,
	.shutdown	= i2c_device_shutdown,
	.pm		= &i2c_device_pm_ops,
};
EXPORT_SYMBOL_GPL(i2c_bus_type);

static struct device_type i2c_client_type = {
	.groups		= i2c_dev_attr_groups,
	.uevent		= i2c_device_uevent,
	.release	= i2c_client_dev_release,
};


struct i2c_client *i2c_verify_client(struct device *dev)
{
	return (dev->type == &i2c_client_type)
			? to_i2c_client(dev)
			: NULL;
}
EXPORT_SYMBOL(i2c_verify_client);


static int i2c_check_client_addr_validity(const struct i2c_client *client)
{
	if (client->flags & I2C_CLIENT_TEN) {
		/* 10-bit address, all values are valid */
		if (client->addr > 0x3ff)
			return -EINVAL;
	} else {
		/* 7-bit address, reject the general call address */
		if (client->addr == 0x00 || client->addr > 0x7f)
			return -EINVAL;
	}
	return 0;
}

static int i2c_check_addr_validity(unsigned short addr)
{
	/*
	 * Reserved addresses per I2C specification:
	 *  0x00       General call address / START byte
	 *  0x01       CBUS address
	 *  0x02       Reserved for different bus format
	 *  0x03       Reserved for future purposes
	 *  0x04-0x07  Hs-mode master code
	 *  0x78-0x7b  10-bit slave addressing
	 *  0x7c-0x7f  Reserved for future purposes
	 */
	if (addr < 0x08 || addr > 0x77)
		return -EINVAL;
	return 0;
}

static int __i2c_check_addr_busy(struct device *dev, void *addrp)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	int			addr = *(int *)addrp;

	if (client && client->addr == addr)
		return -EBUSY;
	return 0;
}

static int i2c_check_addr_busy(struct i2c_adapter *adapter, int addr)
{
	return device_for_each_child(&adapter->dev, &addr,
				     __i2c_check_addr_busy);
}

struct i2c_client *
i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
	struct i2c_client	*client;
	int			status;

	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client)
		return NULL;

	client->adapter = adap;

	client->dev.platform_data = info->platform_data;

	if (info->archdata)
		client->dev.archdata = *info->archdata;

	client->flags = info->flags;
	client->addr = info->addr;
	client->irq = info->irq;

	strlcpy(client->name, info->type, sizeof(client->name));

	/* Check for address validity */
	status = i2c_check_client_addr_validity(client);
	if (status) {
		dev_err(&adap->dev, "Invalid %d-bit I2C address 0x%02hx\n",
			client->flags & I2C_CLIENT_TEN ? 10 : 7, client->addr);
		goto out_err_silent;
	}

	/* Check for address business */
	status = i2c_check_addr_busy(adap, client->addr);
	if (status)
		goto out_err;

	client->dev.parent = &client->adapter->dev;
	client->dev.bus = &i2c_bus_type;
	client->dev.type = &i2c_client_type;
#ifdef CONFIG_OF
	client->dev.of_node = info->of_node;
#endif

	dev_set_name(&client->dev, "%d-%04x", i2c_adapter_id(adap),
		     client->addr);
	status = device_register(&client->dev);
	if (status)
		goto out_err;

	dev_dbg(&adap->dev, "client [%s] registered with bus id %s\n",
		client->name, dev_name(&client->dev));

	return client;

out_err:
	dev_err(&adap->dev, "Failed to register i2c client %s at 0x%02x "
		"(%d)\n", client->name, client->addr, status);
out_err_silent:
	kfree(client);
	return NULL;
}
EXPORT_SYMBOL_GPL(i2c_new_device);


void i2c_unregister_device(struct i2c_client *client)
{
	device_unregister(&client->dev);
}
EXPORT_SYMBOL_GPL(i2c_unregister_device);


static const struct i2c_device_id dummy_id[] = {
	{ "dummy", 0 },
	{ },
};

static int dummy_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	return 0;
}

static int dummy_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver dummy_driver = {
	.driver.name	= "dummy",
	.probe		= dummy_probe,
	.remove		= dummy_remove,
	.id_table	= dummy_id,
};

struct i2c_client *i2c_new_dummy(struct i2c_adapter *adapter, u16 address)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("dummy", address),
	};

	return i2c_new_device(adapter, &info);
}
EXPORT_SYMBOL_GPL(i2c_new_dummy);

/* ------------------------------------------------------------------------- */

/* I2C bus adapters -- one roots each I2C or SMBUS segment */

static void i2c_adapter_dev_release(struct device *dev)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	complete(&adap->dev_released);
}

static ssize_t
i2c_sysfs_new_device(struct device *dev, struct device_attribute *attr,
		     const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct i2c_board_info info;
	struct i2c_client *client;
	char *blank, end;
	int res;

	dev_warn(dev, "The new_device interface is still experimental "
		 "and may change in a near future\n");
	memset(&info, 0, sizeof(struct i2c_board_info));

	blank = strchr(buf, ' ');
	if (!blank) {
		dev_err(dev, "%s: Missing parameters\n", "new_device");
		return -EINVAL;
	}
	if (blank - buf > I2C_NAME_SIZE - 1) {
		dev_err(dev, "%s: Invalid device name\n", "new_device");
		return -EINVAL;
	}
	memcpy(info.type, buf, blank - buf);

	/* Parse remaining parameters, reject extra parameters */
	res = sscanf(++blank, "%hi%c", &info.addr, &end);
	if (res < 1) {
		dev_err(dev, "%s: Can't parse I2C address\n", "new_device");
		return -EINVAL;
	}
	if (res > 1  && end != '\n') {
		dev_err(dev, "%s: Extra parameters\n", "new_device");
		return -EINVAL;
	}

	client = i2c_new_device(adap, &info);
	if (!client)
		return -EINVAL;

	/* Keep track of the added device */
	i2c_lock_adapter(adap);
	list_add_tail(&client->detected, &adap->userspace_clients);
	i2c_unlock_adapter(adap);
	dev_info(dev, "%s: Instantiated device %s at 0x%02hx\n", "new_device",
		 info.type, info.addr);

	return count;
}

static ssize_t
i2c_sysfs_delete_device(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct i2c_client *client, *next;
	unsigned short addr;
	char end;
	int res;

	/* Parse parameters, reject extra parameters */
	res = sscanf(buf, "%hi%c", &addr, &end);
	if (res < 1) {
		dev_err(dev, "%s: Can't parse I2C address\n", "delete_device");
		return -EINVAL;
	}
	if (res > 1  && end != '\n') {
		dev_err(dev, "%s: Extra parameters\n", "delete_device");
		return -EINVAL;
	}

	/* Make sure the device was added through sysfs */
	res = -ENOENT;
	i2c_lock_adapter(adap);
	list_for_each_entry_safe(client, next, &adap->userspace_clients,
				 detected) {
		if (client->addr == addr) {
			dev_info(dev, "%s: Deleting device %s at 0x%02hx\n",
				 "delete_device", client->name, client->addr);

			list_del(&client->detected);
			i2c_unregister_device(client);
			res = count;
			break;
		}
	}
	i2c_unlock_adapter(adap);

	if (res < 0)
		dev_err(dev, "%s: Can't find device in list\n",
			"delete_device");
	return res;
}

static DEVICE_ATTR(new_device, S_IWUSR, NULL, i2c_sysfs_new_device);
static DEVICE_ATTR(delete_device, S_IWUSR, NULL, i2c_sysfs_delete_device);

static struct attribute *i2c_adapter_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_new_device.attr,
	&dev_attr_delete_device.attr,
	NULL
};

static struct attribute_group i2c_adapter_attr_group = {
	.attrs		= i2c_adapter_attrs,
};

static const struct attribute_group *i2c_adapter_attr_groups[] = {
	&i2c_adapter_attr_group,
	NULL
};

static struct device_type i2c_adapter_type = {
	.groups		= i2c_adapter_attr_groups,
	.release	= i2c_adapter_dev_release,
};

#ifdef CONFIG_I2C_COMPAT
static struct class_compat *i2c_adapter_compat_class;
#endif

static void i2c_scan_static_board_info(struct i2c_adapter *adapter)
{
	struct i2c_devinfo	*devinfo;

	down_read(&__i2c_board_lock);
	list_for_each_entry(devinfo, &__i2c_board_list, list) {
		if (devinfo->busnum == adapter->nr
				&& !i2c_new_device(adapter,
						&devinfo->board_info))
			dev_err(&adapter->dev,
				"Can't create device at 0x%02x\n",
				devinfo->board_info.addr);
	}
	up_read(&__i2c_board_lock);
}

static int i2c_do_add_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adap)
{
	/* Detect supported devices on that bus, and instantiate them */
	i2c_detect(adap, driver);

	/* Let legacy drivers scan this bus for matching devices */
	if (driver->attach_adapter) {
		/* We ignore the return code; if it fails, too bad */
		driver->attach_adapter(adap);
	}
	return 0;
}

static int __process_new_adapter(struct device_driver *d, void *data)
{
	return i2c_do_add_adapter(to_i2c_driver(d), data);
}

static int i2c_register_adapter(struct i2c_adapter *adap)
{
	int res = 0, dummy;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!i2c_bus_type.p))) {
		res = -EAGAIN;
		goto out_list;
	}

	rt_mutex_init(&adap->bus_lock);
	INIT_LIST_HEAD(&adap->userspace_clients);

	/* Set default timeout to 1 second if not already set */
	if (adap->timeout == 0)
		adap->timeout = HZ;

	dev_set_name(&adap->dev, "i2c-%d", adap->nr);
	adap->dev.bus = &i2c_bus_type;
	adap->dev.type = &i2c_adapter_type;
	res = device_register(&adap->dev);
	if (res)
		goto out_list;

	dev_dbg(&adap->dev, "adapter [%s] registered\n", adap->name);

#ifdef CONFIG_I2C_COMPAT
	res = class_compat_create_link(i2c_adapter_compat_class, &adap->dev,
				       adap->dev.parent);
	if (res)
		dev_warn(&adap->dev,
			 "Failed to create compatibility class link\n");
#endif

	/* create pre-declared device nodes */
	if (adap->nr < __i2c_first_dynamic_bus_num)
		i2c_scan_static_board_info(adap);

	/* Notify drivers */
	mutex_lock(&core_lock);
	dummy = bus_for_each_drv(&i2c_bus_type, NULL, adap,
				 __process_new_adapter);
	mutex_unlock(&core_lock);

	return 0;

out_list:
	mutex_lock(&core_lock);
	idr_remove(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);
	return res;
}

int i2c_add_adapter(struct i2c_adapter *adapter)
{
	int	id, res = 0;

retry:
	if (idr_pre_get(&i2c_adapter_idr, GFP_KERNEL) == 0)
		return -ENOMEM;

	mutex_lock(&core_lock);
	/* "above" here means "above or equal to", sigh */
	res = idr_get_new_above(&i2c_adapter_idr, adapter,
				__i2c_first_dynamic_bus_num, &id);
	mutex_unlock(&core_lock);

	if (res < 0) {
		if (res == -EAGAIN)
			goto retry;
		return res;
	}

	adapter->nr = id;
	return i2c_register_adapter(adapter);
}
EXPORT_SYMBOL(i2c_add_adapter);

int i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
	int	id;
	int	status;

	if (adap->nr & ~MAX_ID_MASK)
		return -EINVAL;

retry:
	if (idr_pre_get(&i2c_adapter_idr, GFP_KERNEL) == 0)
		return -ENOMEM;

	mutex_lock(&core_lock);
	/* "above" here means "above or equal to", sigh;
	 * we need the "equal to" result to force the result
	 */
	status = idr_get_new_above(&i2c_adapter_idr, adap, adap->nr, &id);
	if (status == 0 && id != adap->nr) {
		status = -EBUSY;
		idr_remove(&i2c_adapter_idr, id);
	}
	mutex_unlock(&core_lock);
	if (status == -EAGAIN)
		goto retry;

	if (status == 0)
		status = i2c_register_adapter(adap);
	return status;
}
EXPORT_SYMBOL_GPL(i2c_add_numbered_adapter);

static int i2c_do_del_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adapter)
{
	struct i2c_client *client, *_n;
	int res;

	/* Remove the devices we created ourselves as the result of hardware
	 * probing (using a driver's detect method) */
	list_for_each_entry_safe(client, _n, &driver->clients, detected) {
		if (client->adapter == adapter) {
			dev_dbg(&adapter->dev, "Removing %s at 0x%x\n",
				client->name, client->addr);
			list_del(&client->detected);
			i2c_unregister_device(client);
		}
	}

	if (!driver->detach_adapter)
		return 0;
	res = driver->detach_adapter(adapter);
	if (res)
		dev_err(&adapter->dev, "detach_adapter failed (%d) "
			"for driver [%s]\n", res, driver->driver.name);
	return res;
}

static int __unregister_client(struct device *dev, void *dummy)
{
	struct i2c_client *client = i2c_verify_client(dev);
	if (client)
		i2c_unregister_device(client);
	return 0;
}

static int __process_removed_adapter(struct device_driver *d, void *data)
{
	return i2c_do_del_adapter(to_i2c_driver(d), data);
}

int i2c_del_adapter(struct i2c_adapter *adap)
{
	int res = 0;
	struct i2c_adapter *found;
	struct i2c_client *client, *next;

	/* First make sure that this adapter was ever added */
	mutex_lock(&core_lock);
	found = idr_find(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);
	if (found != adap) {
		pr_debug("i2c-core: attempting to delete unregistered "
			 "adapter [%s]\n", adap->name);
		return -EINVAL;
	}

	/* Tell drivers about this removal */
	mutex_lock(&core_lock);
	res = bus_for_each_drv(&i2c_bus_type, NULL, adap,
			       __process_removed_adapter);
	mutex_unlock(&core_lock);
	if (res)
		return res;

	/* Remove devices instantiated from sysfs */
	i2c_lock_adapter(adap);
	list_for_each_entry_safe(client, next, &adap->userspace_clients,
				 detected) {
		dev_dbg(&adap->dev, "Removing %s at 0x%x\n", client->name,
			client->addr);
		list_del(&client->detected);
		i2c_unregister_device(client);
	}
	i2c_unlock_adapter(adap);

	/* Detach any active clients. This can't fail, thus we do not
	   checking the returned value. */
	res = device_for_each_child(&adap->dev, NULL, __unregister_client);

#ifdef CONFIG_I2C_COMPAT
	class_compat_remove_link(i2c_adapter_compat_class, &adap->dev,
				 adap->dev.parent);
#endif

	/* device name is gone after device_unregister */
	dev_dbg(&adap->dev, "adapter [%s] unregistered\n", adap->name);

	/* clean up the sysfs representation */
	init_completion(&adap->dev_released);
	device_unregister(&adap->dev);

	/* wait for sysfs to drop all references */
	wait_for_completion(&adap->dev_released);

	/* free bus id */
	mutex_lock(&core_lock);
	idr_remove(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);

	/* Clear the device structure in case this adapter is ever going to be
	   added again */
	memset(&adap->dev, 0, sizeof(adap->dev));

	return 0;
}
EXPORT_SYMBOL(i2c_del_adapter);


/* ------------------------------------------------------------------------- */

static int __process_new_driver(struct device *dev, void *data)
{
	if (dev->type != &i2c_adapter_type)
		return 0;
	return i2c_do_add_adapter(data, to_i2c_adapter(dev));
}


int i2c_register_driver(struct module *owner, struct i2c_driver *driver)
{
	int res;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!i2c_bus_type.p)))
		return -EAGAIN;

	/* add the driver to the list of i2c drivers in the driver core */
	driver->driver.owner = owner;
	driver->driver.bus = &i2c_bus_type;

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound devices.
	 */
	res = driver_register(&driver->driver);
	if (res)
		return res;

	pr_debug("i2c-core: driver [%s] registered\n", driver->driver.name);

	INIT_LIST_HEAD(&driver->clients);
	/* Walk the adapters that are already present */
	mutex_lock(&core_lock);
	bus_for_each_dev(&i2c_bus_type, NULL, driver, __process_new_driver);
	mutex_unlock(&core_lock);

	return 0;
}
EXPORT_SYMBOL(i2c_register_driver);

static int __process_removed_driver(struct device *dev, void *data)
{
	if (dev->type != &i2c_adapter_type)
		return 0;
	return i2c_do_del_adapter(data, to_i2c_adapter(dev));
}

void i2c_del_driver(struct i2c_driver *driver)
{
	mutex_lock(&core_lock);
	bus_for_each_dev(&i2c_bus_type, NULL, driver, __process_removed_driver);
	mutex_unlock(&core_lock);

	driver_unregister(&driver->driver);
	pr_debug("i2c-core: driver [%s] unregistered\n", driver->driver.name);
}
EXPORT_SYMBOL(i2c_del_driver);

/* ------------------------------------------------------------------------- */

struct i2c_client *i2c_use_client(struct i2c_client *client)
{
	if (client && get_device(&client->dev))
		return client;
	return NULL;
}
EXPORT_SYMBOL(i2c_use_client);

void i2c_release_client(struct i2c_client *client)
{
	if (client)
		put_device(&client->dev);
}
EXPORT_SYMBOL(i2c_release_client);

struct i2c_cmd_arg {
	unsigned	cmd;
	void		*arg;
};

static int i2c_cmd(struct device *dev, void *_arg)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_cmd_arg	*arg = _arg;

	if (client && client->driver && client->driver->command)
		client->driver->command(client, arg->cmd, arg->arg);
	return 0;
}

void i2c_clients_command(struct i2c_adapter *adap, unsigned int cmd, void *arg)
{
	struct i2c_cmd_arg	cmd_arg;

	cmd_arg.cmd = cmd;
	cmd_arg.arg = arg;
	device_for_each_child(&adap->dev, &cmd_arg, i2c_cmd);
}
EXPORT_SYMBOL(i2c_clients_command);

static int __init i2c_init(void)
{
	int retval;

	retval = bus_register(&i2c_bus_type);
	if (retval)
		return retval;
#ifdef CONFIG_I2C_COMPAT
	i2c_adapter_compat_class = class_compat_register("i2c-adapter");
	if (!i2c_adapter_compat_class) {
		retval = -ENOMEM;
		goto bus_err;
	}
#endif
	retval = i2c_add_driver(&dummy_driver);
	if (retval)
		goto class_err;
	return 0;

class_err:
#ifdef CONFIG_I2C_COMPAT
	class_compat_unregister(i2c_adapter_compat_class);
bus_err:
#endif
	bus_unregister(&i2c_bus_type);
	return retval;
}

static void __exit i2c_exit(void)
{
	i2c_del_driver(&dummy_driver);
#ifdef CONFIG_I2C_COMPAT
	class_compat_unregister(i2c_adapter_compat_class);
#endif
	bus_unregister(&i2c_bus_type);
}

postcore_initcall(i2c_init);
module_exit(i2c_exit);


int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	unsigned long orig_jiffies;
	int ret, try;

	/* REVISIT the fault reporting model here is weak:
	 *
	 *  - When we get an error after receiving N bytes from a slave,
	 *    there is no way to report "N".
	 *
	 *  - When we get a NAK after transmitting N bytes to a slave,
	 *    there is no way to report "N" ... or to let the master
	 *    continue executing the rest of this combined message, if
	 *    that's the appropriate response.
	 *
	 *  - When for example "num" is two and we successfully complete
	 *    the first message but get an error part way through the
	 *    second, it's unclear whether that should be reported as
	 *    one (discarding status on the second message) or errno
	 *    (discarding status on the first one).
	 */

	if (adap->algo->master_xfer) {
#ifdef DEBUG
		for (ret = 0; ret < num; ret++) {
			dev_dbg(&adap->dev, "master_xfer[%d] %c, addr=0x%02x, "
				"len=%d%s\n", ret, (msgs[ret].flags & I2C_M_RD)
				? 'R' : 'W', msgs[ret].addr, msgs[ret].len,
				(msgs[ret].flags & I2C_M_RECV_LEN) ? "+" : "");
		}
#endif

		if (in_atomic() || irqs_disabled()) {
			ret = rt_mutex_trylock(&adap->bus_lock);
			if (!ret)
				/* I2C activity is ongoing. */
				return -EAGAIN;
		} else {
			rt_mutex_lock(&adap->bus_lock);
		}

		/* Retry automatically on arbitration loss */
		orig_jiffies = jiffies;
		for (ret = 0, try = 0; try <= adap->retries; try++) {
			ret = adap->algo->master_xfer(adap, msgs, num);
			if (ret != -EAGAIN)
				break;
			if (time_after(jiffies, orig_jiffies + adap->timeout))
				break;
		}
		rt_mutex_unlock(&adap->bus_lock);

		return ret;
	} else {
		dev_dbg(&adap->dev, "I2C level transfers not supported\n");
		return -EOPNOTSUPP;
	}
}
EXPORT_SYMBOL(i2c_transfer);

int i2c_master_send(struct i2c_client *client, const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}
EXPORT_SYMBOL(i2c_master_send);

int i2c_master_recv(struct i2c_client *client, char *buf, int count)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;

	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}
EXPORT_SYMBOL(i2c_master_recv);


static int i2c_default_probe(struct i2c_adapter *adap, unsigned short addr)
{
	int err;
	union i2c_smbus_data dummy;

#ifdef CONFIG_X86
	if (addr == 0x73 && (adap->class & I2C_CLASS_HWMON)
	 && i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BYTE_DATA))
		err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0,
				     I2C_SMBUS_BYTE_DATA, &dummy);
	else
#endif
	if ((addr & ~0x07) == 0x30 || (addr & ~0x0f) == 0x50
	 || !i2c_check_functionality(adap, I2C_FUNC_SMBUS_QUICK))
		err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0,
				     I2C_SMBUS_BYTE, &dummy);
	else
		err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_WRITE, 0,
				     I2C_SMBUS_QUICK, NULL);

	return err >= 0;
}

static int i2c_detect_address(struct i2c_client *temp_client,
			      struct i2c_driver *driver)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter = temp_client->adapter;
	int addr = temp_client->addr;
	int err;

	/* Make sure the address is valid */
	err = i2c_check_addr_validity(addr);
	if (err) {
		dev_warn(&adapter->dev, "Invalid probe address 0x%02x\n",
			 addr);
		return err;
	}

	/* Skip if already in use */
	if (i2c_check_addr_busy(adapter, addr))
		return 0;

	/* Make sure there is something at this address */
	if (!i2c_default_probe(adapter, addr))
		return 0;

	/* Finally call the custom detection function */
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = addr;
	err = driver->detect(temp_client, &info);
	if (err) {
		/* -ENODEV is returned if the detection fails. We catch it
		   here as this isn't an error. */
		return err == -ENODEV ? 0 : err;
	}

	/* Consistency check */
	if (info.type[0] == '\0') {
		dev_err(&adapter->dev, "%s detection function provided "
			"no name for 0x%x\n", driver->driver.name,
			addr);
	} else {
		struct i2c_client *client;

		/* Detection succeeded, instantiate the device */
		dev_dbg(&adapter->dev, "Creating %s at 0x%02x\n",
			info.type, info.addr);
		client = i2c_new_device(adapter, &info);
		if (client)
			list_add_tail(&client->detected, &driver->clients);
		else
			dev_err(&adapter->dev, "Failed creating %s at 0x%02x\n",
				info.type, info.addr);
	}
	return 0;
}

static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver)
{
	const unsigned short *address_list;
	struct i2c_client *temp_client;
	int i, err = 0;
	int adap_id = i2c_adapter_id(adapter);

	address_list = driver->address_list;
	if (!driver->detect || !address_list)
		return 0;

	/* Set up a temporary client to help detect callback */
	temp_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!temp_client)
		return -ENOMEM;
	temp_client->adapter = adapter;

	/* Stop here if the classes do not match */
	if (!(adapter->class & driver->class))
		goto exit_free;

	/* Stop here if the bus doesn't support probing */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BYTE)) {
		if (address_list[0] == I2C_CLIENT_END)
			goto exit_free;

		dev_warn(&adapter->dev, "Probing not supported\n");
		err = -EOPNOTSUPP;
		goto exit_free;
	}

	for (i = 0; address_list[i] != I2C_CLIENT_END; i += 1) {
		dev_dbg(&adapter->dev, "found normal entry for adapter %d, "
			"addr 0x%02x\n", adap_id, address_list[i]);
		temp_client->addr = address_list[i];
		err = i2c_detect_address(temp_client, driver);
		if (err)
			goto exit_free;
	}

 exit_free:
	kfree(temp_client);
	return err;
}

struct i2c_client *
i2c_new_probed_device(struct i2c_adapter *adap,
		      struct i2c_board_info *info,
		      unsigned short const *addr_list)
{
	int i;

	/* Stop here if the bus doesn't support probing */
	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BYTE)) {
		dev_err(&adap->dev, "Probing not supported\n");
		return NULL;
	}

	for (i = 0; addr_list[i] != I2C_CLIENT_END; i++) {
		/* Check address validity */
		if (i2c_check_addr_validity(addr_list[i]) < 0) {
			dev_warn(&adap->dev, "Invalid 7-bit address "
				 "0x%02x\n", addr_list[i]);
			continue;
		}

		/* Check address availability */
		if (i2c_check_addr_busy(adap, addr_list[i])) {
			dev_dbg(&adap->dev, "Address 0x%02x already in "
				"use, not probing\n", addr_list[i]);
			continue;
		}

		/* Test address responsiveness */
		if (i2c_default_probe(adap, addr_list[i]))
			break;
	}

	if (addr_list[i] == I2C_CLIENT_END) {
		dev_dbg(&adap->dev, "Probing failed, no device found\n");
		return NULL;
	}

	info->addr = addr_list[i];
	return i2c_new_device(adap, info);
}
EXPORT_SYMBOL_GPL(i2c_new_probed_device);

struct i2c_adapter *i2c_get_adapter(int id)
{
	struct i2c_adapter *adapter;

	mutex_lock(&core_lock);
	adapter = idr_find(&i2c_adapter_idr, id);
	if (adapter && !try_module_get(adapter->owner))
		adapter = NULL;

	mutex_unlock(&core_lock);
	return adapter;
}
EXPORT_SYMBOL(i2c_get_adapter);

void i2c_put_adapter(struct i2c_adapter *adap)
{
	module_put(adap->owner);
}
EXPORT_SYMBOL(i2c_put_adapter);

/* The SMBus parts */

#define POLY    (0x1070U << 3)
static u8 crc8(u16 data)
{
	int i;

	for (i = 0; i < 8; i++) {
		if (data & 0x8000)
			data = data ^ POLY;
		data = data << 1;
	}
	return (u8)(data >> 8);
}

/* Incremental CRC8 over count bytes in the array pointed to by p */
static u8 i2c_smbus_pec(u8 crc, u8 *p, size_t count)
{
	int i;

	for (i = 0; i < count; i++)
		crc = crc8((crc ^ p[i]) << 8);
	return crc;
}

/* Assume a 7-bit address, which is reasonable for SMBus */
static u8 i2c_smbus_msg_pec(u8 pec, struct i2c_msg *msg)
{
	/* The address will be sent first */
	u8 addr = (msg->addr << 1) | !!(msg->flags & I2C_M_RD);
	pec = i2c_smbus_pec(pec, &addr, 1);

	/* The data buffer follows */
	return i2c_smbus_pec(pec, msg->buf, msg->len);
}

/* Used for write only transactions */
static inline void i2c_smbus_add_pec(struct i2c_msg *msg)
{
	msg->buf[msg->len] = i2c_smbus_msg_pec(0, msg);
	msg->len++;
}

static int i2c_smbus_check_pec(u8 cpec, struct i2c_msg *msg)
{
	u8 rpec = msg->buf[--msg->len];
	cpec = i2c_smbus_msg_pec(cpec, msg);

	if (rpec != cpec) {
		pr_debug("i2c-core: Bad PEC 0x%02x vs. 0x%02x\n",
			rpec, cpec);
		return -EBADMSG;
	}
	return 0;
}

s32 i2c_smbus_read_byte(struct i2c_client *client)
{
	union i2c_smbus_data data;
	int status;

	status = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				I2C_SMBUS_READ, 0,
				I2C_SMBUS_BYTE, &data);
	return (status < 0) ? status : data.byte;
}
EXPORT_SYMBOL(i2c_smbus_read_byte);

s32 i2c_smbus_write_byte(struct i2c_client *client, u8 value)
{
	return i2c_smbus_xfer(client->adapter, client->addr, client->flags,
	                      I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, NULL);
}
EXPORT_SYMBOL(i2c_smbus_write_byte);

s32 i2c_smbus_read_byte_data(struct i2c_client *client, u8 command)
{
	union i2c_smbus_data data;
	int status;

	status = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				I2C_SMBUS_READ, command,
				I2C_SMBUS_BYTE_DATA, &data);
	return (status < 0) ? status : data.byte;
}
EXPORT_SYMBOL(i2c_smbus_read_byte_data);

s32 i2c_smbus_write_byte_data(struct i2c_client *client, u8 command, u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			      I2C_SMBUS_WRITE, command,
			      I2C_SMBUS_BYTE_DATA, &data);
}
EXPORT_SYMBOL(i2c_smbus_write_byte_data);

s32 i2c_smbus_read_word_data(struct i2c_client *client, u8 command)
{
	union i2c_smbus_data data;
	int status;

	status = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				I2C_SMBUS_READ, command,
				I2C_SMBUS_WORD_DATA, &data);
	return (status < 0) ? status : data.word;
}
EXPORT_SYMBOL(i2c_smbus_read_word_data);

s32 i2c_smbus_write_word_data(struct i2c_client *client, u8 command, u16 value)
{
	union i2c_smbus_data data;
	data.word = value;
	return i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			      I2C_SMBUS_WRITE, command,
			      I2C_SMBUS_WORD_DATA, &data);
}
EXPORT_SYMBOL(i2c_smbus_write_word_data);

s32 i2c_smbus_process_call(struct i2c_client *client, u8 command, u16 value)
{
	union i2c_smbus_data data;
	int status;
	data.word = value;

	status = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				I2C_SMBUS_WRITE, command,
				I2C_SMBUS_PROC_CALL, &data);
	return (status < 0) ? status : data.word;
}
EXPORT_SYMBOL(i2c_smbus_process_call);

s32 i2c_smbus_read_block_data(struct i2c_client *client, u8 command,
			      u8 *values)
{
	union i2c_smbus_data data;
	int status;

	status = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				I2C_SMBUS_READ, command,
				I2C_SMBUS_BLOCK_DATA, &data);
	if (status)
		return status;

	memcpy(values, &data.block[1], data.block[0]);
	return data.block[0];
}
EXPORT_SYMBOL(i2c_smbus_read_block_data);

s32 i2c_smbus_write_block_data(struct i2c_client *client, u8 command,
			       u8 length, const u8 *values)
{
	union i2c_smbus_data data;

	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;
	memcpy(&data.block[1], values, length);
	return i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			      I2C_SMBUS_WRITE, command,
			      I2C_SMBUS_BLOCK_DATA, &data);
}
EXPORT_SYMBOL(i2c_smbus_write_block_data);

/* Returns the number of read bytes */
s32 i2c_smbus_read_i2c_block_data(struct i2c_client *client, u8 command,
				  u8 length, u8 *values)
{
	union i2c_smbus_data data;
	int status;

	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;
	status = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				I2C_SMBUS_READ, command,
				I2C_SMBUS_I2C_BLOCK_DATA, &data);
	if (status < 0)
		return status;

	memcpy(values, &data.block[1], data.block[0]);
	return data.block[0];
}
EXPORT_SYMBOL(i2c_smbus_read_i2c_block_data);

s32 i2c_smbus_write_i2c_block_data(struct i2c_client *client, u8 command,
				   u8 length, const u8 *values)
{
	union i2c_smbus_data data;

	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;
	memcpy(data.block + 1, values, length);
	return i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			      I2C_SMBUS_WRITE, command,
			      I2C_SMBUS_I2C_BLOCK_DATA, &data);
}
EXPORT_SYMBOL(i2c_smbus_write_i2c_block_data);

static s32 i2c_smbus_xfer_emulated(struct i2c_adapter *adapter, u16 addr,
				   unsigned short flags,
				   char read_write, u8 command, int size,
				   union i2c_smbus_data *data)
{
	/* So we need to generate a series of msgs. In the case of writing, we
	  need to use only one message; when reading, we need two. We initialize
	  most things with sane defaults, to keep the code below somewhat
	  simpler. */
	unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX+3];
	unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX+2];
	int num = read_write == I2C_SMBUS_READ ? 2 : 1;
	struct i2c_msg msg[2] = { { addr, flags, 1, msgbuf0 },
	                          { addr, flags | I2C_M_RD, 0, msgbuf1 }
	                        };
	int i;
	u8 partial_pec = 0;
	int status;

	msgbuf0[0] = command;
	switch (size) {
	case I2C_SMBUS_QUICK:
		msg[0].len = 0;
		/* Special case: The read/write field is used as data */
		msg[0].flags = flags | (read_write == I2C_SMBUS_READ ?
					I2C_M_RD : 0);
		num = 1;
		break;
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_READ) {
			/* Special case: only a read! */
			msg[0].flags = I2C_M_RD | flags;
			num = 1;
		}
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 1;
		else {
			msg[0].len = 2;
			msgbuf0[1] = data->byte;
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 2;
		else {
			msg[0].len = 3;
			msgbuf0[1] = data->word & 0xff;
			msgbuf0[2] = data->word >> 8;
		}
		break;
	case I2C_SMBUS_PROC_CALL:
		num = 2; /* Special case */
		read_write = I2C_SMBUS_READ;
		msg[0].len = 3;
		msg[1].len = 2;
		msgbuf0[1] = data->word & 0xff;
		msgbuf0[2] = data->word >> 8;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].flags |= I2C_M_RECV_LEN;
			msg[1].len = 1; /* block length will be added by
					   the underlying bus driver */
		} else {
			msg[0].len = data->block[0] + 2;
			if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 2) {
				dev_err(&adapter->dev,
					"Invalid block write size %d\n",
					data->block[0]);
				return -EINVAL;
			}
			for (i = 1; i < msg[0].len; i++)
				msgbuf0[i] = data->block[i-1];
		}
		break;
	case I2C_SMBUS_BLOCK_PROC_CALL:
		num = 2; /* Another special case */
		read_write = I2C_SMBUS_READ;
		if (data->block[0] > I2C_SMBUS_BLOCK_MAX) {
			dev_err(&adapter->dev,
				"Invalid block write size %d\n",
				data->block[0]);
			return -EINVAL;
		}
		msg[0].len = data->block[0] + 2;
		for (i = 1; i < msg[0].len; i++)
			msgbuf0[i] = data->block[i-1];
		msg[1].flags |= I2C_M_RECV_LEN;
		msg[1].len = 1; /* block length will be added by
				   the underlying bus driver */
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].len = data->block[0];
		} else {
			msg[0].len = data->block[0] + 1;
			if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 1) {
				dev_err(&adapter->dev,
					"Invalid block write size %d\n",
					data->block[0]);
				return -EINVAL;
			}
			for (i = 1; i <= data->block[0]; i++)
				msgbuf0[i] = data->block[i];
		}
		break;
	default:
		dev_err(&adapter->dev, "Unsupported transaction %d\n", size);
		return -EOPNOTSUPP;
	}

	i = ((flags & I2C_CLIENT_PEC) && size != I2C_SMBUS_QUICK
				      && size != I2C_SMBUS_I2C_BLOCK_DATA);
	if (i) {
		/* Compute PEC if first message is a write */
		if (!(msg[0].flags & I2C_M_RD)) {
			if (num == 1) /* Write only */
				i2c_smbus_add_pec(&msg[0]);
			else /* Write followed by read */
				partial_pec = i2c_smbus_msg_pec(0, &msg[0]);
		}
		/* Ask for PEC if last message is a read */
		if (msg[num-1].flags & I2C_M_RD)
			msg[num-1].len++;
	}

	status = i2c_transfer(adapter, msg, num);
	if (status < 0)
		return status;

	/* Check PEC if last message is a read */
	if (i && (msg[num-1].flags & I2C_M_RD)) {
		status = i2c_smbus_check_pec(partial_pec, &msg[num-1]);
		if (status < 0)
			return status;
	}

	if (read_write == I2C_SMBUS_READ)
		switch (size) {
		case I2C_SMBUS_BYTE:
			data->byte = msgbuf0[0];
			break;
		case I2C_SMBUS_BYTE_DATA:
			data->byte = msgbuf1[0];
			break;
		case I2C_SMBUS_WORD_DATA:
		case I2C_SMBUS_PROC_CALL:
			data->word = msgbuf1[0] | (msgbuf1[1] << 8);
			break;
		case I2C_SMBUS_I2C_BLOCK_DATA:
			for (i = 0; i < data->block[0]; i++)
				data->block[i+1] = msgbuf1[i];
			break;
		case I2C_SMBUS_BLOCK_DATA:
		case I2C_SMBUS_BLOCK_PROC_CALL:
			for (i = 0; i < msgbuf1[0] + 1; i++)
				data->block[i] = msgbuf1[i];
			break;
		}
	return 0;
}

s32 i2c_smbus_xfer(struct i2c_adapter *adapter, u16 addr, unsigned short flags,
		   char read_write, u8 command, int protocol,
		   union i2c_smbus_data *data)
{
	unsigned long orig_jiffies;
	int try;
	s32 res;

	flags &= I2C_M_TEN | I2C_CLIENT_PEC;

	if (adapter->algo->smbus_xfer) {
		rt_mutex_lock(&adapter->bus_lock);

		/* Retry automatically on arbitration loss */
		orig_jiffies = jiffies;
		for (res = 0, try = 0; try <= adapter->retries; try++) {
			res = adapter->algo->smbus_xfer(adapter, addr, flags,
							read_write, command,
							protocol, data);
			if (res != -EAGAIN)
				break;
			if (time_after(jiffies,
				       orig_jiffies + adapter->timeout))
				break;
		}
		rt_mutex_unlock(&adapter->bus_lock);
	} else
		res = i2c_smbus_xfer_emulated(adapter, addr, flags, read_write,
					      command, protocol, data);

	return res;
}
EXPORT_SYMBOL(i2c_smbus_xfer);

MODULE_AUTHOR("Simon G. Vogl <simon@tk.uni-linz.ac.at>");
MODULE_DESCRIPTION("I2C-Bus main module");
MODULE_LICENSE("GPL");
