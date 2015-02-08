

#include <linux/kobject.h>
#include <linux/device.h>
#include "base.h"

struct kobject *hypervisor_kobj;
EXPORT_SYMBOL_GPL(hypervisor_kobj);

int __init hypervisor_init(void)
{
	hypervisor_kobj = kobject_create_and_add("hypervisor", NULL);
	if (!hypervisor_kobj)
		return -ENOMEM;
	return 0;
}
