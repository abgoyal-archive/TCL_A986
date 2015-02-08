

#ifndef __BFA_DEFS_PM_H__
#define __BFA_DEFS_PM_H__

#include <bfa_os_inc.h>

enum bfa_pm_ds {
	BFA_PM_DS_D0 = 0,	/*  full power mode */
	BFA_PM_DS_D1 = 1,	/*  power save state 1 */
	BFA_PM_DS_D2 = 2,	/*  power save state 2 */
	BFA_PM_DS_D3 = 3,	/*  power off state */
};

#endif /* __BFA_DEFS_PM_H__ */
