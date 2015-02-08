

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/common.h>

int __init mxc_register_device(struct platform_device *pdev, void *data)
{
	int ret;

	pdev->dev.platform_data = data;

	ret = platform_device_register(pdev);
	if (ret)
		pr_debug("Unable to register platform device '%s': %d\n",
			 pdev->name, ret);

	return ret;
}

