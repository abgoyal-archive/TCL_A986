
#ifndef _LINUX_OF_PLATFORM_H
#define _LINUX_OF_PLATFORM_H

#ifdef CONFIG_OF_DEVICE
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/pm.h>
#include <linux/of_device.h>

extern struct bus_type of_platform_bus_type;

struct of_platform_driver
{
	int	(*probe)(struct of_device* dev,
			 const struct of_device_id *match);
	int	(*remove)(struct of_device* dev);

	int	(*suspend)(struct of_device* dev, pm_message_t state);
	int	(*resume)(struct of_device* dev);
	int	(*shutdown)(struct of_device* dev);

	struct device_driver	driver;
};
#define	to_of_platform_driver(drv) \
	container_of(drv,struct of_platform_driver, driver)

extern int of_register_driver(struct of_platform_driver *drv,
			      struct bus_type *bus);
extern void of_unregister_driver(struct of_platform_driver *drv);

/* Platform drivers register/unregister */
static inline int of_register_platform_driver(struct of_platform_driver *drv)
{
	return of_register_driver(drv, &of_platform_bus_type);
}
static inline void of_unregister_platform_driver(struct of_platform_driver *drv)
{
	of_unregister_driver(drv);
}

#include <asm/of_platform.h>

extern struct of_device *of_find_device_by_node(struct device_node *np);

extern int of_bus_type_init(struct bus_type *bus, const char *name);
#endif /* CONFIG_OF_DEVICE */

#endif	/* _LINUX_OF_PLATFORM_H */
