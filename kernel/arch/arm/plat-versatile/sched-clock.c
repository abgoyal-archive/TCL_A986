
#include <linux/cnt32_to_63.h>
#include <linux/io.h>
#include <asm/div64.h>

#include <mach/hardware.h>
#include <mach/platform.h>

#ifdef VERSATILE_SYS_BASE
#define REFCOUNTER	(__io_address(VERSATILE_SYS_BASE) + VERSATILE_SYS_24MHz_OFFSET)
#endif

#ifdef REALVIEW_SYS_BASE
#define REFCOUNTER	(__io_address(REALVIEW_SYS_BASE) + REALVIEW_SYS_24MHz_OFFSET)
#endif

unsigned long long sched_clock(void)
{
	unsigned long long v = cnt32_to_63(readl(REFCOUNTER));

	/* the <<1 gets rid of the cnt_32_to_63 top bit saving on a bic insn */
	v *= 125<<1;
	do_div(v, 3<<1);

	return v;
}
