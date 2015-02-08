

#include <asm/system.h>
#include <asm/iommu.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

void do_after_copyback(void)
{
	iommu_restore();
	touch_softlockup_watchdog();
	mb();
}

void _iommu_save(void)
{
	iommu_save();
}
