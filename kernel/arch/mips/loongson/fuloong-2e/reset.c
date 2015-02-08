

#include <loongson.h>

void mach_prepare_reboot(void)
{
	LOONGSON_GENCFG &= ~(1 << 2);
	LOONGSON_GENCFG |= (1 << 2);
}

void mach_prepare_shutdown(void)
{
}
