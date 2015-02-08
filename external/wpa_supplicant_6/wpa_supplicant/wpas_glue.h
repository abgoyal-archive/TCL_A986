

#ifndef WPAS_GLUE_H
#define WPAS_GLUE_H

int wpa_supplicant_init_eapol(struct wpa_supplicant *wpa_s);
int wpa_supplicant_init_wpa(struct wpa_supplicant *wpa_s);
void wpa_supplicant_rsn_supp_set_config(struct wpa_supplicant *wpa_s,
					struct wpa_ssid *ssid);

#ifdef CONFIG_WAPI_SUPPORT
int wpa_supplicant_init_wapi(struct wpa_supplicant *wpa_s);
int wpa_supplicant_deinit_wapi(struct wpa_supplicant *wpa_s);
#endif

#endif /* WPAS_GLUE_H */
