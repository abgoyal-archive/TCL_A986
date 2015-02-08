

#ifndef _ASM_EBUS_H
#define _ASM_EBUS_H
#ifdef __KERNEL__

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

extern struct bus_type ibmebus_bus_type;

int ibmebus_register_driver(struct of_platform_driver *drv);
void ibmebus_unregister_driver(struct of_platform_driver *drv);

int ibmebus_request_irq(u32 ist, irq_handler_t handler,
			unsigned long irq_flags, const char *devname,
			void *dev_id);
void ibmebus_free_irq(u32 ist, void *dev_id);

#endif /* __KERNEL__ */
#endif /* _ASM_IBMEBUS_H */
