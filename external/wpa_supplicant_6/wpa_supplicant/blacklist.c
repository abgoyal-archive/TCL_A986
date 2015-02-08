

#include "includes.h"

#include "common.h"
#include "wpa_supplicant_i.h"
#include "blacklist.h"

struct wpa_blacklist * wpa_blacklist_get(struct wpa_supplicant *wpa_s,
					 const u8 *bssid)
{
	struct wpa_blacklist *e;

	e = wpa_s->blacklist;
	while (e) {
		if (os_memcmp(e->bssid, bssid, ETH_ALEN) == 0)
			return e;
		e = e->next;
	}

	return NULL;
}


int wpa_blacklist_add(struct wpa_supplicant *wpa_s, const u8 *bssid)
{
	struct wpa_blacklist *e;

	e = wpa_blacklist_get(wpa_s, bssid);
	if (e) {
		e->count++;
		wpa_printf(MSG_DEBUG, "BSSID " MACSTR " blacklist count "
			   "incremented to %d",
			   MAC2STR(bssid), e->count);
		return 0;
	}

	e = os_zalloc(sizeof(*e));
	if (e == NULL)
		return -1;
	os_memcpy(e->bssid, bssid, ETH_ALEN);
	e->count = 1;
	e->next = wpa_s->blacklist;
	wpa_s->blacklist = e;
	wpa_printf(MSG_DEBUG, "Added BSSID " MACSTR " into blacklist",
		   MAC2STR(bssid));

	return 0;
}


int wpa_blacklist_del(struct wpa_supplicant *wpa_s, const u8 *bssid)
{
	struct wpa_blacklist *e, *prev = NULL;

	e = wpa_s->blacklist;
	while (e) {
		if (os_memcmp(e->bssid, bssid, ETH_ALEN) == 0) {
			if (prev == NULL) {
				wpa_s->blacklist = e->next;
			} else {
				prev->next = e->next;
			}
			wpa_printf(MSG_DEBUG, "Removed BSSID " MACSTR " from "
				   "blacklist", MAC2STR(bssid));
			os_free(e);
			return 0;
		}
		prev = e;
		e = e->next;
	}
	return -1;
}


void wpa_blacklist_clear(struct wpa_supplicant *wpa_s)
{
	struct wpa_blacklist *e, *prev;

	e = wpa_s->blacklist;
	wpa_s->blacklist = NULL;
	while (e) {
		prev = e;
		e = e->next;
		wpa_printf(MSG_DEBUG, "Removed BSSID " MACSTR " from "
			   "blacklist (clear)", MAC2STR(prev->bssid));
		os_free(prev);
	}
}
