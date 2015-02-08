

#include <linux/init.h>
#include <linux/acpi.h>
#include <acpi/acpi_drivers.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "internal.h"
#include "sleep.h"

#define _COMPONENT		ACPI_SYSTEM_COMPONENT
ACPI_MODULE_NAME("wakeup_devices")

void acpi_enable_wakeup_device_prep(u8 sleep_state)
{
	struct list_head *node, *next;

	list_for_each_safe(node, next, &acpi_wakeup_device_list) {
		struct acpi_device *dev = container_of(node,
						       struct acpi_device,
						       wakeup_list);

		if (!dev->wakeup.flags.valid || !dev->wakeup.state.enabled
		    || (sleep_state > (u32) dev->wakeup.sleep_state))
			continue;

		acpi_enable_wakeup_device_power(dev, sleep_state);
	}
}

void acpi_enable_wakeup_device(u8 sleep_state)
{
	struct list_head *node, *next;

	/* 
	 * Caution: this routine must be invoked when interrupt is disabled 
	 * Refer ACPI2.0: P212
	 */
	list_for_each_safe(node, next, &acpi_wakeup_device_list) {
		struct acpi_device *dev =
			container_of(node, struct acpi_device, wakeup_list);

		if (!dev->wakeup.flags.valid || !dev->wakeup.state.enabled
		    || sleep_state > (u32) dev->wakeup.sleep_state)
			continue;

		/* The wake-up power should have been enabled already. */
		acpi_enable_gpe(dev->wakeup.gpe_device, dev->wakeup.gpe_number,
				ACPI_GPE_TYPE_WAKE);
	}
}

void acpi_disable_wakeup_device(u8 sleep_state)
{
	struct list_head *node, *next;

	list_for_each_safe(node, next, &acpi_wakeup_device_list) {
		struct acpi_device *dev =
			container_of(node, struct acpi_device, wakeup_list);

		if (!dev->wakeup.flags.valid || !dev->wakeup.state.enabled
		    || (sleep_state > (u32) dev->wakeup.sleep_state))
			continue;

		acpi_disable_gpe(dev->wakeup.gpe_device, dev->wakeup.gpe_number,
				ACPI_GPE_TYPE_WAKE);
		acpi_disable_wakeup_device_power(dev);
	}
}

int __init acpi_wakeup_device_init(void)
{
	struct list_head *node, *next;

	mutex_lock(&acpi_device_lock);
	list_for_each_safe(node, next, &acpi_wakeup_device_list) {
		struct acpi_device *dev = container_of(node,
						       struct acpi_device,
						       wakeup_list);
		if (dev->wakeup.flags.always_enabled)
			dev->wakeup.state.enabled = 1;
	}
	mutex_unlock(&acpi_device_lock);
	return 0;
}
