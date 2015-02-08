
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/amba/bus.h>

#include <asm/irq.h>
#include <asm/sizes.h>

#define to_amba_device(d)	container_of(d, struct amba_device, dev)
#define to_amba_driver(d)	container_of(d, struct amba_driver, drv)

static struct amba_id *
amba_lookup(struct amba_id *table, struct amba_device *dev)
{
	int ret = 0;

	while (table->mask) {
		ret = (dev->periphid & table->mask) == table->id;
		if (ret)
			break;
		table++;
	}

	return ret ? table : NULL;
}

static int amba_match(struct device *dev, struct device_driver *drv)
{
	struct amba_device *pcdev = to_amba_device(dev);
	struct amba_driver *pcdrv = to_amba_driver(drv);

	return amba_lookup(pcdrv->id_table, pcdev) != NULL;
}

#ifdef CONFIG_HOTPLUG
static int amba_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct amba_device *pcdev = to_amba_device(dev);
	int retval = 0;

	retval = add_uevent_var(env, "AMBA_ID=%08x", pcdev->periphid);
	return retval;
}
#else
#define amba_uevent NULL
#endif

static int amba_suspend(struct device *dev, pm_message_t state)
{
	struct amba_driver *drv = to_amba_driver(dev->driver);
	int ret = 0;

	if (dev->driver && drv->suspend)
		ret = drv->suspend(to_amba_device(dev), state);
	return ret;
}

static int amba_resume(struct device *dev)
{
	struct amba_driver *drv = to_amba_driver(dev->driver);
	int ret = 0;

	if (dev->driver && drv->resume)
		ret = drv->resume(to_amba_device(dev));
	return ret;
}

#define amba_attr_func(name,fmt,arg...)					\
static ssize_t name##_show(struct device *_dev,				\
			   struct device_attribute *attr, char *buf)	\
{									\
	struct amba_device *dev = to_amba_device(_dev);			\
	return sprintf(buf, fmt, arg);					\
}

#define amba_attr(name,fmt,arg...)	\
amba_attr_func(name,fmt,arg)		\
static DEVICE_ATTR(name, S_IRUGO, name##_show, NULL)

amba_attr_func(id, "%08x\n", dev->periphid);
amba_attr(irq0, "%u\n", dev->irq[0]);
amba_attr(irq1, "%u\n", dev->irq[1]);
amba_attr_func(resource, "\t%016llx\t%016llx\t%016lx\n",
	 (unsigned long long)dev->res.start, (unsigned long long)dev->res.end,
	 dev->res.flags);

static struct device_attribute amba_dev_attrs[] = {
	__ATTR_RO(id),
	__ATTR_RO(resource),
	__ATTR_NULL,
};

static struct bus_type amba_bustype = {
	.name		= "amba",
	.dev_attrs	= amba_dev_attrs,
	.match		= amba_match,
	.uevent		= amba_uevent,
	.suspend	= amba_suspend,
	.resume		= amba_resume,
};

static int __init amba_init(void)
{
	return bus_register(&amba_bustype);
}

postcore_initcall(amba_init);

static int amba_probe(struct device *dev)
{
	struct amba_device *pcdev = to_amba_device(dev);
	struct amba_driver *pcdrv = to_amba_driver(dev->driver);
	struct amba_id *id;

	id = amba_lookup(pcdrv->id_table, pcdev);

	return pcdrv->probe(pcdev, id);
}

static int amba_remove(struct device *dev)
{
	struct amba_driver *drv = to_amba_driver(dev->driver);
	return drv->remove(to_amba_device(dev));
}

static void amba_shutdown(struct device *dev)
{
	struct amba_driver *drv = to_amba_driver(dev->driver);
	drv->shutdown(to_amba_device(dev));
}

int amba_driver_register(struct amba_driver *drv)
{
	drv->drv.bus = &amba_bustype;

#define SETFN(fn)	if (drv->fn) drv->drv.fn = amba_##fn
	SETFN(probe);
	SETFN(remove);
	SETFN(shutdown);

	return driver_register(&drv->drv);
}

void amba_driver_unregister(struct amba_driver *drv)
{
	driver_unregister(&drv->drv);
}


static void amba_device_release(struct device *dev)
{
	struct amba_device *d = to_amba_device(dev);

	if (d->res.parent)
		release_resource(&d->res);
	kfree(d);
}

int amba_device_register(struct amba_device *dev, struct resource *parent)
{
	u32 pid, cid;
	u32 size;
	void __iomem *tmp;
	int i, ret;

	device_initialize(&dev->dev);

	/*
	 * Copy from device_add
	 */
	if (dev->dev.init_name) {
		dev_set_name(&dev->dev, "%s", dev->dev.init_name);
		dev->dev.init_name = NULL;
	}

	dev->dev.release = amba_device_release;
	dev->dev.bus = &amba_bustype;
	dev->dev.dma_mask = &dev->dma_mask;
	dev->res.name = dev_name(&dev->dev);

	if (!dev->dev.coherent_dma_mask && dev->dma_mask)
		dev_warn(&dev->dev, "coherent dma mask is unset\n");

	ret = request_resource(parent, &dev->res);
	if (ret)
		goto err_out;

	/*
	 * Dynamically calculate the size of the resource
	 * and use this for iomap
	 */
	size = resource_size(&dev->res);
	tmp = ioremap(dev->res.start, size);
	if (!tmp) {
		ret = -ENOMEM;
		goto err_release;
	}

	/*
	 * Read pid and cid based on size of resource
	 * they are located at end of region
	 */
	for (pid = 0, i = 0; i < 4; i++)
		pid |= (readl(tmp + size - 0x20 + 4 * i) & 255) << (i * 8);
	for (cid = 0, i = 0; i < 4; i++)
		cid |= (readl(tmp + size - 0x10 + 4 * i) & 255) << (i * 8);

	iounmap(tmp);

	if (cid == 0xb105f00d)
		dev->periphid = pid;

	if (!dev->periphid) {
		ret = -ENODEV;
		goto err_release;
	}

	ret = device_add(&dev->dev);
	if (ret)
		goto err_release;

	if (dev->irq[0] != NO_IRQ)
		ret = device_create_file(&dev->dev, &dev_attr_irq0);
	if (ret == 0 && dev->irq[1] != NO_IRQ)
		ret = device_create_file(&dev->dev, &dev_attr_irq1);
	if (ret == 0)
		return ret;

	device_unregister(&dev->dev);

 err_release:
	release_resource(&dev->res);
 err_out:
	return ret;
}

void amba_device_unregister(struct amba_device *dev)
{
	device_unregister(&dev->dev);
}


struct find_data {
	struct amba_device *dev;
	struct device *parent;
	const char *busid;
	unsigned int id;
	unsigned int mask;
};

static int amba_find_match(struct device *dev, void *data)
{
	struct find_data *d = data;
	struct amba_device *pcdev = to_amba_device(dev);
	int r;

	r = (pcdev->periphid & d->mask) == d->id;
	if (d->parent)
		r &= d->parent == dev->parent;
	if (d->busid)
		r &= strcmp(dev_name(dev), d->busid) == 0;

	if (r) {
		get_device(dev);
		d->dev = pcdev;
	}

	return r;
}

struct amba_device *
amba_find_device(const char *busid, struct device *parent, unsigned int id,
		 unsigned int mask)
{
	struct find_data data;

	data.dev = NULL;
	data.parent = parent;
	data.busid = busid;
	data.id = id;
	data.mask = mask;

	bus_for_each_dev(&amba_bustype, NULL, &data, amba_find_match);

	return data.dev;
}

int amba_request_regions(struct amba_device *dev, const char *name)
{
	int ret = 0;
	u32 size;

	if (!name)
		name = dev->dev.driver->name;

	size = resource_size(&dev->res);

	if (!request_mem_region(dev->res.start, size, name))
		ret = -EBUSY;

	return ret;
}

void amba_release_regions(struct amba_device *dev)
{
	u32 size;

	size = resource_size(&dev->res);
	release_mem_region(dev->res.start, size);
}

EXPORT_SYMBOL(amba_driver_register);
EXPORT_SYMBOL(amba_driver_unregister);
EXPORT_SYMBOL(amba_device_register);
EXPORT_SYMBOL(amba_device_unregister);
EXPORT_SYMBOL(amba_find_device);
EXPORT_SYMBOL(amba_request_regions);
EXPORT_SYMBOL(amba_release_regions);
