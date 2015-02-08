

#include <linux/kernel.h>
#include <linux/types.h>

struct platform_device; /* don't need the contents */

#include <plat/iic.h>

void s3c_i2c0_cfg_gpio(struct platform_device *dev)
{
	/* Will be populated later */
}
