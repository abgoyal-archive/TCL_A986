

#include <xen/xen-ops.h>
#include <asm/xen/hypervisor.h>
#include "time.h"

void
xen_mm_pin_all(void)
{
	/* nothing */
}

void
xen_mm_unpin_all(void)
{
	/* nothing */
}

void xen_pre_device_suspend(void)
{
	/* nothing */
}

void
xen_pre_suspend()
{
	/* nothing */
}

void
xen_post_suspend(int suspend_cancelled)
{
	if (suspend_cancelled)
		return;

	xen_ia64_enable_opt_feature();
	/* add more if necessary */
}

void xen_arch_resume(void)
{
	xen_timer_resume_on_aps();
}
