

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/acpi.h>
#include <acpi/acpi_bus.h>
#include <acpi/acpi_drivers.h>
#include <acpi/hed.h>

static struct acpi_device_id acpi_hed_ids[] = {
	{"PNP0C33", 0},
	{"", 0},
};
MODULE_DEVICE_TABLE(acpi, acpi_hed_ids);

static acpi_handle hed_handle;

static BLOCKING_NOTIFIER_HEAD(acpi_hed_notify_list);

int register_acpi_hed_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&acpi_hed_notify_list, nb);
}
EXPORT_SYMBOL_GPL(register_acpi_hed_notifier);

void unregister_acpi_hed_notifier(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&acpi_hed_notify_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_acpi_hed_notifier);

static void acpi_hed_notify(struct acpi_device *device, u32 event)
{
	blocking_notifier_call_chain(&acpi_hed_notify_list, 0, NULL);
}

static int __devinit acpi_hed_add(struct acpi_device *device)
{
	/* Only one hardware error device */
	if (hed_handle)
		return -EINVAL;
	hed_handle = device->handle;
	return 0;
}

static int __devexit acpi_hed_remove(struct acpi_device *device, int type)
{
	hed_handle = NULL;
	return 0;
}

static struct acpi_driver acpi_hed_driver = {
	.name = "hardware_error_device",
	.class = "hardware_error",
	.ids = acpi_hed_ids,
	.ops = {
		.add = acpi_hed_add,
		.remove = acpi_hed_remove,
		.notify = acpi_hed_notify,
	},
};

static int __init acpi_hed_init(void)
{
	if (acpi_disabled)
		return -ENODEV;

	if (acpi_bus_register_driver(&acpi_hed_driver) < 0)
		return -ENODEV;

	return 0;
}

static void __exit acpi_hed_exit(void)
{
	acpi_bus_unregister_driver(&acpi_hed_driver);
}

module_init(acpi_hed_init);
module_exit(acpi_hed_exit);

ACPI_MODULE_NAME("hed");
MODULE_AUTHOR("Huang Ying");
MODULE_DESCRIPTION("ACPI Hardware Error Device Driver");
MODULE_LICENSE("GPL");
