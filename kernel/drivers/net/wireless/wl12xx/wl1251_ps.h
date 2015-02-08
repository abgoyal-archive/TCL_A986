
#ifndef __WL1251_PS_H__
#define __WL1251_PS_H__


#include "wl1251.h"
#include "wl1251_acx.h"

int wl1251_ps_set_mode(struct wl1251 *wl, enum wl1251_cmd_ps_mode mode);
void wl1251_ps_elp_sleep(struct wl1251 *wl);
int wl1251_ps_elp_wakeup(struct wl1251 *wl);
void wl1251_elp_work(struct work_struct *work);


#endif /* __WL1251_PS_H__ */
