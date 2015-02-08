
#ifndef __NET_WIRELESS_REG_H
#define __NET_WIRELESS_REG_H

extern const struct ieee80211_regdomain *cfg80211_regdomain;

bool is_world_regdom(const char *alpha2);
bool reg_is_valid_request(const char *alpha2);

int regulatory_hint_user(const char *alpha2);

void reg_device_remove(struct wiphy *wiphy);

int regulatory_init(void);
void regulatory_exit(void);

int set_regdom(const struct ieee80211_regdomain *rd);

int regulatory_hint_found_beacon(struct wiphy *wiphy,
					struct ieee80211_channel *beacon_chan,
					gfp_t gfp);

void regulatory_hint_11d(struct wiphy *wiphy,
			 enum ieee80211_band band,
			 u8 *country_ie,
			 u8 country_ie_len);

void regulatory_hint_disconnect(void);

#endif  /* __NET_WIRELESS_REG_H */
