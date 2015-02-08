

/* this file (will) contains wireless extension handlers*/

#ifndef R8180_WX_H
#define R8180_WX_H
//#include <linux/wireless.h>
//#include "ieee80211.h"
extern struct iw_handler_def r8192_wx_handlers_def;
/* Enable  the rtl819x_core.c to share this function, david 2008.9.22 */
extern struct iw_statistics *r8192_get_wireless_stats(struct net_device *dev);

#endif
