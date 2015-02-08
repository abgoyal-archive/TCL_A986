

#include "ath.h"
#include "debug.h"

void ath_print(struct ath_common *common, int dbg_mask, const char *fmt, ...)
{
	va_list args;

	if (likely(!(common->debug_mask & dbg_mask)))
		return;

	va_start(args, fmt);
	printk(KERN_DEBUG "ath: ");
	vprintk(fmt, args);
	va_end(args);
}
EXPORT_SYMBOL(ath_print);
