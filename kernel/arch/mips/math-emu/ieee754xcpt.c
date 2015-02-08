


#include <linux/kernel.h>
#include "ieee754.h"


static const char *const rtnames[] = {
	"sp", "dp", "xp", "si", "di"
};

void ieee754_xcpt(struct ieee754xctx *xcp)
{
	printk(KERN_DEBUG "floating point exception in \"%s\", type=%s\n",
		xcp->op, rtnames[xcp->rt]);
}
