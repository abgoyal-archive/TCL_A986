

#ifndef WPS_SUPPLICANT_H
#define WPS_SUPPLICANT_H

#ifdef CONFIG_WPS

#include "wps/wps.h"
#include "wps/wps_defs.h"


struct wps_new_ap_settings {
	const char *ssid_hex;
	const char *auth;
	const char *encr;
	const char *key_hex;
};

u16 wps_fix_config_methods(u16 config_methods);
int wpas_wps_init(struct wpa_supplicant *wpa_s);
void wpas_wps_deinit(struct wpa_supplicant *wpa_s);
int wpas_wps_eapol_cb(struct wpa_supplicant *wpa_s);
enum wps_request_type wpas_wps_get_req_type(struct wpa_ssid *ssid);
int wpas_wps_start_pbc(struct wpa_supplicant *wpa_s, const u8 *bssid);
int wpas_wps_start_pin(struct wpa_supplicant *wpa_s, const u8 *bssid,
		       const char *pin);
int wpas_wps_start_reg(struct wpa_supplicant *wpa_s, const u8 *bssid,
		       const char *pin, struct wps_new_ap_settings *settings);
void wpas_clear_wps(struct wpa_supplicant *wpa_s);
int wpas_wps_ssid_bss_match(struct wpa_supplicant *wpa_s,
			    struct wpa_ssid *ssid, struct wpa_scan_res *bss);
int wpas_wps_ssid_wildcard_ok(struct wpa_supplicant *wpa_s,
			      struct wpa_ssid *ssid, struct wpa_scan_res *bss);
int wpas_wps_scan_pbc_overlap(struct wpa_supplicant *wpa_s,
			      struct wpa_scan_res *selected,
			      struct wpa_ssid *ssid);
void wpas_wps_notify_scan_results(struct wpa_supplicant *wpa_s);
int wpas_wps_searching(struct wpa_supplicant *wpa_s);

#else /* CONFIG_WPS */

static inline int wpas_wps_init(struct wpa_supplicant *wpa_s)
{
	return 0;
}

static inline void wpas_wps_deinit(struct wpa_supplicant *wpa_s)
{
}

static inline int wpas_wps_eapol_cb(struct wpa_supplicant *wpa_s)
{
	return 0;
}

static inline u8 wpas_wps_get_req_type(struct wpa_ssid *ssid)
{
	return 0;
}

static inline int wpas_wps_ssid_bss_match(struct wpa_supplicant *wpa_s,
					  struct wpa_ssid *ssid,
					  struct wpa_scan_res *bss)
{
	return -1;
}

static inline int wpas_wps_ssid_wildcard_ok(struct wpa_supplicant *wpa_s,
					    struct wpa_ssid *ssid,
					    struct wpa_scan_res *bss)
{
	return 0;
}

static inline int wpas_wps_scan_pbc_overlap(struct wpa_supplicant *wpa_s,
					    struct wpa_scan_res *selected,
					    struct wpa_ssid *ssid)
{
	return 0;
}

static inline void wpas_wps_notify_scan_results(struct wpa_supplicant *wpa_s)
{
}

static inline int wpas_wps_searching(struct wpa_supplicant *wpa_s)
{
	return 0;
}

#endif /* CONFIG_WPS */

#endif /* WPS_SUPPLICANT_H */
