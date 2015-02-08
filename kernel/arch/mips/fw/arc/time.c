
#include <linux/init.h>

#include <asm/fw/arc/types.h>
#include <asm/sgialib.h>

struct linux_tinfo * __init
ArcGetTime(VOID)
{
	return (struct linux_tinfo *) ARC_CALL0(get_tinfo);
}

ULONG __init
ArcGetRelativeTime(VOID)
{
	return ARC_CALL0(get_rtime);
}
