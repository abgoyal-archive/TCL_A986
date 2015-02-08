

#ifndef __iwl_spectrum_h__
#define __iwl_spectrum_h__
enum {				/* ieee80211_basic_report.map */
	IEEE80211_BASIC_MAP_BSS = (1 << 0),
	IEEE80211_BASIC_MAP_OFDM = (1 << 1),
	IEEE80211_BASIC_MAP_UNIDENTIFIED = (1 << 2),
	IEEE80211_BASIC_MAP_RADAR = (1 << 3),
	IEEE80211_BASIC_MAP_UNMEASURED = (1 << 4),
	/* Bits 5-7 are reserved */

};
struct ieee80211_basic_report {
	u8 channel;
	__le64 start_time;
	__le16 duration;
	u8 map;
} __attribute__ ((packed));

enum {				/* ieee80211_measurement_request.mode */
	/* Bit 0 is reserved */
	IEEE80211_MEASUREMENT_ENABLE = (1 << 1),
	IEEE80211_MEASUREMENT_REQUEST = (1 << 2),
	IEEE80211_MEASUREMENT_REPORT = (1 << 3),
	/* Bits 4-7 are reserved */
};

enum {
	IEEE80211_REPORT_BASIC = 0,	/* required */
	IEEE80211_REPORT_CCA = 1,	/* optional */
	IEEE80211_REPORT_RPI = 2,	/* optional */
	/* 3-255 reserved */
};

struct ieee80211_measurement_params {
	u8 channel;
	__le64 start_time;
	__le16 duration;
} __attribute__ ((packed));

struct ieee80211_info_element {
	u8 id;
	u8 len;
	u8 data[0];
} __attribute__ ((packed));

struct ieee80211_measurement_request {
	struct ieee80211_info_element ie;
	u8 token;
	u8 mode;
	u8 type;
	struct ieee80211_measurement_params params[0];
} __attribute__ ((packed));

struct ieee80211_measurement_report {
	struct ieee80211_info_element ie;
	u8 token;
	u8 mode;
	u8 type;
	union {
		struct ieee80211_basic_report basic[0];
	} u;
} __attribute__ ((packed));

#endif
