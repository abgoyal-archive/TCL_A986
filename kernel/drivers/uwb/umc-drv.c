
#include <linux/kernel.h>
#include <linux/uwb/umc.h>

int __umc_driver_register(struct umc_driver *umc_drv, struct module *module,
			  const char *mod_name)
{
	umc_drv->driver.name     = umc_drv->name;
	umc_drv->driver.owner    = module;
	umc_drv->driver.mod_name = mod_name;
	umc_drv->driver.bus      = &umc_bus_type;

	return driver_register(&umc_drv->driver);
}
EXPORT_SYMBOL_GPL(__umc_driver_register);

void umc_driver_unregister(struct umc_driver *umc_drv)
{
	driver_unregister(&umc_drv->driver);
}
EXPORT_SYMBOL_GPL(umc_driver_unregister);
