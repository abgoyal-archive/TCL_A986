

#include <linux/module.h>
#include <asm/param.h>
#include <asm/delay.h>

EXPORT_SYMBOL(udelay);

void udelay(unsigned long usecs)
{
	_udelay(usecs);
}

