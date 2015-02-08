

#define __GENERATING_BOUNDS_H
/* Include headers that define the enum constants of interest */
#include <linux/page-flags.h>
#include <linux/mmzone.h>
#include <linux/kbuild.h>

void foo(void)
{
	/* The enum constants to put into include/generated/bounds.h */
	DEFINE(NR_PAGEFLAGS, __NR_PAGEFLAGS);
	DEFINE(MAX_NR_ZONES, __MAX_NR_ZONES);
	/* End of constants */
}
