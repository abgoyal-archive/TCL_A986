

#ifndef __HOSTAP_H__
#define __HOSTAP_H__

#include "device.h"

/*---------------------  Export Definitions -------------------------*/

#define WLAN_RATE_1M    BIT0
#define WLAN_RATE_2M    BIT1
#define WLAN_RATE_5M5   BIT2
#define WLAN_RATE_11M   BIT3
#define WLAN_RATE_6M    BIT4
#define WLAN_RATE_9M    BIT5
#define WLAN_RATE_12M   BIT6
#define WLAN_RATE_18M   BIT7
#define WLAN_RATE_24M   BIT8
#define WLAN_RATE_36M   BIT9
#define WLAN_RATE_48M   BIT10
#define WLAN_RATE_54M   BIT11


/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

#ifndef ETH_P_PAE
#define ETH_P_PAE 0x888E /* Port Access Entity (IEEE 802.1X) */
#endif /* ETH_P_PAE */

#ifndef ARPHRD_IEEE80211
#define ARPHRD_IEEE80211 801
#endif

int vt6656_hostap_set_hostapd(PSDevice pDevice, int val, int rtnl_locked);
int vt6656_hostap_ioctl(PSDevice pDevice, struct iw_point *p);

#endif /* __HOSTAP_H__ */
