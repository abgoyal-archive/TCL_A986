

#ifndef __WL1271_PS_H__
#define __WL1271_PS_H__

#include "wl1271.h"
#include "wl1271_acx.h"

int wl1271_ps_set_mode(struct wl1271 *wl, enum wl1271_cmd_ps_mode mode,
		       bool send);
void wl1271_ps_elp_sleep(struct wl1271 *wl);
int wl1271_ps_elp_wakeup(struct wl1271 *wl, bool chip_awake);
void wl1271_elp_work(struct work_struct *work);

#endif /* __WL1271_PS_H__ */
