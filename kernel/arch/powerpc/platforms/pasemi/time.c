

#include <linux/time.h>

#include <asm/time.h>

unsigned long __init pas_get_boot_time(void)
{
	/* Let's just return a fake date right now */
	return mktime(2006, 1, 1, 12, 0, 0);
}
