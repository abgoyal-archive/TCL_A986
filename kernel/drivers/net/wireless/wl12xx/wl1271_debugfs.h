

#ifndef WL1271_DEBUGFS_H
#define WL1271_DEBUGFS_H

#include "wl1271.h"

int wl1271_debugfs_init(struct wl1271 *wl);
void wl1271_debugfs_exit(struct wl1271 *wl);
void wl1271_debugfs_reset(struct wl1271 *wl);

#endif /* WL1271_DEBUGFS_H */
