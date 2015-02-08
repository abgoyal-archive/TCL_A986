
#include <linux/init.h>
#include <linux/kernel.h>

#include <asm/bcache.h>

#include <asm/fw/arc/types.h>
#include <asm/sgialib.h>
#include <asm/bootinfo.h>
#include <asm/system.h>

VOID
ArcHalt(VOID)
{
	bc_disable();
	local_irq_disable();
	ARC_CALL0(halt);
never:	goto never;
}

VOID
ArcPowerDown(VOID)
{
	bc_disable();
	local_irq_disable();
	ARC_CALL0(pdown);
never:	goto never;
}

/* XXX is this a soft reset basically? XXX */
VOID
ArcRestart(VOID)
{
	bc_disable();
	local_irq_disable();
	ARC_CALL0(restart);
never:	goto never;
}

VOID
ArcReboot(VOID)
{
	bc_disable();
	local_irq_disable();
	ARC_CALL0(reboot);
never:	goto never;
}

VOID
ArcEnterInteractiveMode(VOID)
{
	bc_disable();
	local_irq_disable();
	ARC_CALL0(imode);
never:	goto never;
}

LONG
ArcSaveConfiguration(VOID)
{
	return ARC_CALL0(cfg_save);
}

struct linux_sysid *
ArcGetSystemId(VOID)
{
	return (struct linux_sysid *) ARC_CALL0(get_sysid);
}

VOID __init
ArcFlushAllCaches(VOID)
{
	ARC_CALL0(cache_flush);
}

DISPLAY_STATUS * __init ArcGetDisplayStatus(ULONG FileID)
{
	return (DISPLAY_STATUS *) ARC_CALL1(GetDisplayStatus, FileID);
}
