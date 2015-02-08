

#ifndef BLACKLIST_H
#define BLACKLIST_H

struct wpa_blacklist {
	struct wpa_blacklist *next;
	u8 bssid[ETH_ALEN];
	int count;
};

struct wpa_blacklist * wpa_blacklist_get(struct wpa_supplicant *wpa_s,
					 const u8 *bssid);
int wpa_blacklist_add(struct wpa_supplicant *wpa_s, const u8 *bssid);
int wpa_blacklist_del(struct wpa_supplicant *wpa_s, const u8 *bssid);
void wpa_blacklist_clear(struct wpa_supplicant *wpa_s);

#endif /* BLACKLIST_H */
