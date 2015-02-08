

#ifndef _P80211MSG_H
#define _P80211MSG_H

#define WLAN_DEVNAMELEN_MAX	16

typedef struct p80211msg {
	u32 msgcode;
	u32 msglen;
	u8 devname[WLAN_DEVNAMELEN_MAX];
} __attribute__ ((packed)) p80211msg_t;

#endif /* _P80211MSG_H */
