
#ifndef __NET_REGULATORY_H
#define __NET_REGULATORY_H


enum environment_cap {
	ENVIRON_ANY,
	ENVIRON_INDOOR,
	ENVIRON_OUTDOOR,
};

struct regulatory_request {
	int wiphy_idx;
	enum nl80211_reg_initiator initiator;
	char alpha2[2];
	bool intersect;
	u32 country_ie_checksum;
	enum environment_cap country_ie_env;
	struct list_head list;
};

struct ieee80211_freq_range {
	u32 start_freq_khz;
	u32 end_freq_khz;
	u32 max_bandwidth_khz;
};

struct ieee80211_power_rule {
	u32 max_antenna_gain;
	u32 max_eirp;
};

struct ieee80211_reg_rule {
	struct ieee80211_freq_range freq_range;
	struct ieee80211_power_rule power_rule;
	u32 flags;
};

struct ieee80211_regdomain {
	u32 n_reg_rules;
	char alpha2[2];
	struct ieee80211_reg_rule reg_rules[];
};

#define MHZ_TO_KHZ(freq) ((freq) * 1000)
#define KHZ_TO_MHZ(freq) ((freq) / 1000)
#define DBI_TO_MBI(gain) ((gain) * 100)
#define MBI_TO_DBI(gain) ((gain) / 100)
#define DBM_TO_MBM(gain) ((gain) * 100)
#define MBM_TO_DBM(gain) ((gain) / 100)

#define REG_RULE(start, end, bw, gain, eirp, reg_flags) \
{							\
	.freq_range.start_freq_khz = MHZ_TO_KHZ(start),	\
	.freq_range.end_freq_khz = MHZ_TO_KHZ(end),	\
	.freq_range.max_bandwidth_khz = MHZ_TO_KHZ(bw),	\
	.power_rule.max_antenna_gain = DBI_TO_MBI(gain),\
	.power_rule.max_eirp = DBM_TO_MBM(eirp),	\
	.flags = reg_flags,				\
}

#endif
