

#ifndef __WL1271_INIT_H__
#define __WL1271_INIT_H__

#include "wl1271.h"

int wl1271_hw_init_power_auth(struct wl1271 *wl);
int wl1271_init_templates_config(struct wl1271 *wl);
int wl1271_init_phy_config(struct wl1271 *wl);
int wl1271_init_pta(struct wl1271 *wl);
int wl1271_init_energy_detection(struct wl1271 *wl);
int wl1271_hw_init(struct wl1271 *wl);

#endif
