
#ifndef __NET_CFG80211_H
#define __NET_CFG80211_H

#include <linux/netdevice.h>
#include <linux/debugfs.h>
#include <linux/list.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/nl80211.h>
#include <linux/if_ether.h>
#include <linux/ieee80211.h>
#include <net/regulatory.h>

/* remove once we remove the wext stuff */
#include <net/iw_handler.h>
#include <linux/wireless.h>



enum ieee80211_band {
	IEEE80211_BAND_2GHZ = NL80211_BAND_2GHZ,
	IEEE80211_BAND_5GHZ = NL80211_BAND_5GHZ,

	/* keep last */
	IEEE80211_NUM_BANDS
};

enum ieee80211_channel_flags {
	IEEE80211_CHAN_DISABLED		= 1<<0,
	IEEE80211_CHAN_PASSIVE_SCAN	= 1<<1,
	IEEE80211_CHAN_NO_IBSS		= 1<<2,
	IEEE80211_CHAN_RADAR		= 1<<3,
	IEEE80211_CHAN_NO_HT40PLUS	= 1<<4,
	IEEE80211_CHAN_NO_HT40MINUS	= 1<<5,
};

#define IEEE80211_CHAN_NO_HT40 \
	(IEEE80211_CHAN_NO_HT40PLUS | IEEE80211_CHAN_NO_HT40MINUS)

struct ieee80211_channel {
	enum ieee80211_band band;
	u16 center_freq;
	u16 hw_value;
	u32 flags;
	int max_antenna_gain;
	int max_power;
	bool beacon_found;
	u32 orig_flags;
	int orig_mag, orig_mpwr;
};

enum ieee80211_rate_flags {
	IEEE80211_RATE_SHORT_PREAMBLE	= 1<<0,
	IEEE80211_RATE_MANDATORY_A	= 1<<1,
	IEEE80211_RATE_MANDATORY_B	= 1<<2,
	IEEE80211_RATE_MANDATORY_G	= 1<<3,
	IEEE80211_RATE_ERP_G		= 1<<4,
};

struct ieee80211_rate {
	u32 flags;
	u16 bitrate;
	u16 hw_value, hw_value_short;
};

struct ieee80211_sta_ht_cap {
	u16 cap; /* use IEEE80211_HT_CAP_ */
	bool ht_supported;
	u8 ampdu_factor;
	u8 ampdu_density;
	struct ieee80211_mcs_info mcs;
};

struct ieee80211_supported_band {
	struct ieee80211_channel *channels;
	struct ieee80211_rate *bitrates;
	enum ieee80211_band band;
	int n_channels;
	int n_bitrates;
	struct ieee80211_sta_ht_cap ht_cap;
};


struct vif_params {
       u8 *mesh_id;
       int mesh_id_len;
       int use_4addr;
};

struct key_params {
	u8 *key;
	u8 *seq;
	int key_len;
	int seq_len;
	u32 cipher;
};

enum survey_info_flags {
	SURVEY_INFO_NOISE_DBM = 1<<0,
};

struct survey_info {
	struct ieee80211_channel *channel;
	u32 filled;
	s8 noise;
};

struct beacon_parameters {
	u8 *head, *tail;
	int interval, dtim_period;
	int head_len, tail_len;
};

enum plink_actions {
	PLINK_ACTION_INVALID,
	PLINK_ACTION_OPEN,
	PLINK_ACTION_BLOCK,
};

struct station_parameters {
	u8 *supported_rates;
	struct net_device *vlan;
	u32 sta_flags_mask, sta_flags_set;
	int listen_interval;
	u16 aid;
	u8 supported_rates_len;
	u8 plink_action;
	struct ieee80211_ht_cap *ht_capa;
};

enum station_info_flags {
	STATION_INFO_INACTIVE_TIME	= 1<<0,
	STATION_INFO_RX_BYTES		= 1<<1,
	STATION_INFO_TX_BYTES		= 1<<2,
	STATION_INFO_LLID		= 1<<3,
	STATION_INFO_PLID		= 1<<4,
	STATION_INFO_PLINK_STATE	= 1<<5,
	STATION_INFO_SIGNAL		= 1<<6,
	STATION_INFO_TX_BITRATE		= 1<<7,
	STATION_INFO_RX_PACKETS		= 1<<8,
	STATION_INFO_TX_PACKETS		= 1<<9,
};

enum rate_info_flags {
	RATE_INFO_FLAGS_MCS		= 1<<0,
	RATE_INFO_FLAGS_40_MHZ_WIDTH	= 1<<1,
	RATE_INFO_FLAGS_SHORT_GI	= 1<<2,
};

struct rate_info {
	u8 flags;
	u8 mcs;
	u16 legacy;
};

struct station_info {
	u32 filled;
	u32 inactive_time;
	u32 rx_bytes;
	u32 tx_bytes;
	u16 llid;
	u16 plid;
	u8 plink_state;
	s8 signal;
	struct rate_info txrate;
	u32 rx_packets;
	u32 tx_packets;

	int generation;
};

enum monitor_flags {
	MONITOR_FLAG_FCSFAIL		= 1<<NL80211_MNTR_FLAG_FCSFAIL,
	MONITOR_FLAG_PLCPFAIL		= 1<<NL80211_MNTR_FLAG_PLCPFAIL,
	MONITOR_FLAG_CONTROL		= 1<<NL80211_MNTR_FLAG_CONTROL,
	MONITOR_FLAG_OTHER_BSS		= 1<<NL80211_MNTR_FLAG_OTHER_BSS,
	MONITOR_FLAG_COOK_FRAMES	= 1<<NL80211_MNTR_FLAG_COOK_FRAMES,
};

enum mpath_info_flags {
	MPATH_INFO_FRAME_QLEN		= BIT(0),
	MPATH_INFO_SN			= BIT(1),
	MPATH_INFO_METRIC		= BIT(2),
	MPATH_INFO_EXPTIME		= BIT(3),
	MPATH_INFO_DISCOVERY_TIMEOUT	= BIT(4),
	MPATH_INFO_DISCOVERY_RETRIES	= BIT(5),
	MPATH_INFO_FLAGS		= BIT(6),
};

struct mpath_info {
	u32 filled;
	u32 frame_qlen;
	u32 sn;
	u32 metric;
	u32 exptime;
	u32 discovery_timeout;
	u8 discovery_retries;
	u8 flags;

	int generation;
};

struct bss_parameters {
	int use_cts_prot;
	int use_short_preamble;
	int use_short_slot_time;
	u8 *basic_rates;
	u8 basic_rates_len;
	int ap_isolate;
};

struct mesh_config {
	/* Timeouts in ms */
	/* Mesh plink management parameters */
	u16 dot11MeshRetryTimeout;
	u16 dot11MeshConfirmTimeout;
	u16 dot11MeshHoldingTimeout;
	u16 dot11MeshMaxPeerLinks;
	u8  dot11MeshMaxRetries;
	u8  dot11MeshTTL;
	bool auto_open_plinks;
	/* HWMP parameters */
	u8  dot11MeshHWMPmaxPREQretries;
	u32 path_refresh_time;
	u16 min_discovery_timeout;
	u32 dot11MeshHWMPactivePathTimeout;
	u16 dot11MeshHWMPpreqMinInterval;
	u16 dot11MeshHWMPnetDiameterTraversalTime;
	u8  dot11MeshHWMPRootMode;
};

struct ieee80211_txq_params {
	enum nl80211_txq_q queue;
	u16 txop;
	u16 cwmin;
	u16 cwmax;
	u8 aifs;
};

/* from net/wireless.h */
struct wiphy;

/* from net/ieee80211.h */
struct ieee80211_channel;

struct cfg80211_ssid {
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 ssid_len;
};

struct cfg80211_scan_request {
	struct cfg80211_ssid *ssids;
	int n_ssids;
	u32 n_channels;
	const u8 *ie;
	size_t ie_len;

	/* internal */
	struct wiphy *wiphy;
	struct net_device *dev;
	bool aborted;

	/* keep last */
	struct ieee80211_channel *channels[0];
};

enum cfg80211_signal_type {
	CFG80211_SIGNAL_TYPE_NONE,
	CFG80211_SIGNAL_TYPE_MBM,
	CFG80211_SIGNAL_TYPE_UNSPEC,
};

struct cfg80211_bss {
	struct ieee80211_channel *channel;

	u8 bssid[ETH_ALEN];
	u64 tsf;
	u16 beacon_interval;
	u16 capability;
	u8 *information_elements;
	size_t len_information_elements;
	u8 *beacon_ies;
	size_t len_beacon_ies;
	u8 *proberesp_ies;
	size_t len_proberesp_ies;

	s32 signal;

	void (*free_priv)(struct cfg80211_bss *bss);
	u8 priv[0] __attribute__((__aligned__(sizeof(void *))));
};

const u8 *ieee80211_bss_get_ie(struct cfg80211_bss *bss, u8 ie);


struct cfg80211_crypto_settings {
	u32 wpa_versions;
	u32 cipher_group;
	int n_ciphers_pairwise;
	u32 ciphers_pairwise[NL80211_MAX_NR_CIPHER_SUITES];
	int n_akm_suites;
	u32 akm_suites[NL80211_MAX_NR_AKM_SUITES];
	bool control_port;
};

struct cfg80211_auth_request {
	struct cfg80211_bss *bss;
	const u8 *ie;
	size_t ie_len;
	enum nl80211_auth_type auth_type;
	const u8 *key;
	u8 key_len, key_idx;
	bool local_state_change;
};

struct cfg80211_assoc_request {
	struct cfg80211_bss *bss;
	const u8 *ie, *prev_bssid;
	size_t ie_len;
	struct cfg80211_crypto_settings crypto;
	bool use_mfp;
};

struct cfg80211_deauth_request {
	struct cfg80211_bss *bss;
	const u8 *ie;
	size_t ie_len;
	u16 reason_code;
	bool local_state_change;
};

struct cfg80211_disassoc_request {
	struct cfg80211_bss *bss;
	const u8 *ie;
	size_t ie_len;
	u16 reason_code;
	bool local_state_change;
};

struct cfg80211_ibss_params {
	u8 *ssid;
	u8 *bssid;
	struct ieee80211_channel *channel;
	u8 *ie;
	u8 ssid_len, ie_len;
	u16 beacon_interval;
	bool channel_fixed;
	bool privacy;
};

struct cfg80211_connect_params {
	struct ieee80211_channel *channel;
	u8 *bssid;
	u8 *ssid;
	size_t ssid_len;
	enum nl80211_auth_type auth_type;
	u8 *ie;
	size_t ie_len;
	bool privacy;
	struct cfg80211_crypto_settings crypto;
	const u8 *key;
	u8 key_len, key_idx;
};

enum wiphy_params_flags {
	WIPHY_PARAM_RETRY_SHORT		= 1 << 0,
	WIPHY_PARAM_RETRY_LONG		= 1 << 1,
	WIPHY_PARAM_FRAG_THRESHOLD	= 1 << 2,
	WIPHY_PARAM_RTS_THRESHOLD	= 1 << 3,
	WIPHY_PARAM_COVERAGE_CLASS	= 1 << 4,
};

enum tx_power_setting {
	TX_POWER_AUTOMATIC,
	TX_POWER_LIMITED,
	TX_POWER_FIXED,
};

struct cfg80211_bitrate_mask {
	struct {
		u32 legacy;
		/* TODO: add support for masking MCS rates; e.g.: */
		/* u8 mcs[IEEE80211_HT_MCS_MASK_LEN]; */
	} control[IEEE80211_NUM_BANDS];
};
struct cfg80211_pmksa {
	u8 *bssid;
	u8 *pmkid;
};

struct cfg80211_ops {
	int	(*suspend)(struct wiphy *wiphy);
	int	(*resume)(struct wiphy *wiphy);

	int	(*add_virtual_intf)(struct wiphy *wiphy, char *name,
				    enum nl80211_iftype type, u32 *flags,
				    struct vif_params *params);
	int	(*del_virtual_intf)(struct wiphy *wiphy, struct net_device *dev);
	int	(*change_virtual_intf)(struct wiphy *wiphy,
				       struct net_device *dev,
				       enum nl80211_iftype type, u32 *flags,
				       struct vif_params *params);

	int	(*add_key)(struct wiphy *wiphy, struct net_device *netdev,
			   u8 key_index, const u8 *mac_addr,
			   struct key_params *params);
	int	(*get_key)(struct wiphy *wiphy, struct net_device *netdev,
			   u8 key_index, const u8 *mac_addr, void *cookie,
			   void (*callback)(void *cookie, struct key_params*));
	int	(*del_key)(struct wiphy *wiphy, struct net_device *netdev,
			   u8 key_index, const u8 *mac_addr);
	int	(*set_default_key)(struct wiphy *wiphy,
				   struct net_device *netdev,
				   u8 key_index);
	int	(*set_default_mgmt_key)(struct wiphy *wiphy,
					struct net_device *netdev,
					u8 key_index);

	int	(*add_beacon)(struct wiphy *wiphy, struct net_device *dev,
			      struct beacon_parameters *info);
	int	(*set_beacon)(struct wiphy *wiphy, struct net_device *dev,
			      struct beacon_parameters *info);
	int	(*del_beacon)(struct wiphy *wiphy, struct net_device *dev);


	int	(*add_station)(struct wiphy *wiphy, struct net_device *dev,
			       u8 *mac, struct station_parameters *params);
	int	(*del_station)(struct wiphy *wiphy, struct net_device *dev,
			       u8 *mac);
	int	(*change_station)(struct wiphy *wiphy, struct net_device *dev,
				  u8 *mac, struct station_parameters *params);
	int	(*get_station)(struct wiphy *wiphy, struct net_device *dev,
			       u8 *mac, struct station_info *sinfo);
	int	(*dump_station)(struct wiphy *wiphy, struct net_device *dev,
			       int idx, u8 *mac, struct station_info *sinfo);

	int	(*add_mpath)(struct wiphy *wiphy, struct net_device *dev,
			       u8 *dst, u8 *next_hop);
	int	(*del_mpath)(struct wiphy *wiphy, struct net_device *dev,
			       u8 *dst);
	int	(*change_mpath)(struct wiphy *wiphy, struct net_device *dev,
				  u8 *dst, u8 *next_hop);
	int	(*get_mpath)(struct wiphy *wiphy, struct net_device *dev,
			       u8 *dst, u8 *next_hop,
			       struct mpath_info *pinfo);
	int	(*dump_mpath)(struct wiphy *wiphy, struct net_device *dev,
			       int idx, u8 *dst, u8 *next_hop,
			       struct mpath_info *pinfo);
	int	(*get_mesh_params)(struct wiphy *wiphy,
				struct net_device *dev,
				struct mesh_config *conf);
	int	(*set_mesh_params)(struct wiphy *wiphy,
				struct net_device *dev,
				const struct mesh_config *nconf, u32 mask);
	int	(*change_bss)(struct wiphy *wiphy, struct net_device *dev,
			      struct bss_parameters *params);

	int	(*set_txq_params)(struct wiphy *wiphy,
				  struct ieee80211_txq_params *params);

	int	(*set_channel)(struct wiphy *wiphy, struct net_device *dev,
			       struct ieee80211_channel *chan,
			       enum nl80211_channel_type channel_type);

	int	(*scan)(struct wiphy *wiphy, struct net_device *dev,
			struct cfg80211_scan_request *request);

	int	(*auth)(struct wiphy *wiphy, struct net_device *dev,
			struct cfg80211_auth_request *req);
	int	(*assoc)(struct wiphy *wiphy, struct net_device *dev,
			 struct cfg80211_assoc_request *req);
	int	(*deauth)(struct wiphy *wiphy, struct net_device *dev,
			  struct cfg80211_deauth_request *req,
			  void *cookie);
	int	(*disassoc)(struct wiphy *wiphy, struct net_device *dev,
			    struct cfg80211_disassoc_request *req,
			    void *cookie);

	int	(*connect)(struct wiphy *wiphy, struct net_device *dev,
			   struct cfg80211_connect_params *sme);
	int	(*disconnect)(struct wiphy *wiphy, struct net_device *dev,
			      u16 reason_code);

	int	(*join_ibss)(struct wiphy *wiphy, struct net_device *dev,
			     struct cfg80211_ibss_params *params);
	int	(*leave_ibss)(struct wiphy *wiphy, struct net_device *dev);

	int	(*set_wiphy_params)(struct wiphy *wiphy, u32 changed);

	int	(*set_tx_power)(struct wiphy *wiphy,
				enum tx_power_setting type, int dbm);
	int	(*get_tx_power)(struct wiphy *wiphy, int *dbm);

	int	(*set_wds_peer)(struct wiphy *wiphy, struct net_device *dev,
				u8 *addr);

	void	(*rfkill_poll)(struct wiphy *wiphy);

#ifdef CONFIG_NL80211_TESTMODE
	int	(*testmode_cmd)(struct wiphy *wiphy, void *data, int len);
#endif

	int	(*set_bitrate_mask)(struct wiphy *wiphy,
				    struct net_device *dev,
				    const u8 *peer,
				    const struct cfg80211_bitrate_mask *mask);

	int	(*dump_survey)(struct wiphy *wiphy, struct net_device *netdev,
			int idx, struct survey_info *info);

	int	(*set_pmksa)(struct wiphy *wiphy, struct net_device *netdev,
			     struct cfg80211_pmksa *pmksa);
	int	(*del_pmksa)(struct wiphy *wiphy, struct net_device *netdev,
			     struct cfg80211_pmksa *pmksa);
	int	(*flush_pmksa)(struct wiphy *wiphy, struct net_device *netdev);

	int	(*remain_on_channel)(struct wiphy *wiphy,
				     struct net_device *dev,
				     struct ieee80211_channel *chan,
				     enum nl80211_channel_type channel_type,
				     unsigned int duration,
				     u64 *cookie);
	int	(*cancel_remain_on_channel)(struct wiphy *wiphy,
					    struct net_device *dev,
					    u64 cookie);

	int	(*action)(struct wiphy *wiphy, struct net_device *dev,
			  struct ieee80211_channel *chan,
			  enum nl80211_channel_type channel_type,
			  const u8 *buf, size_t len, u64 *cookie);

	int	(*set_power_mgmt)(struct wiphy *wiphy, struct net_device *dev,
				  bool enabled, int timeout);

	int	(*set_cqm_rssi_config)(struct wiphy *wiphy,
				       struct net_device *dev,
				       s32 rssi_thold, u32 rssi_hyst);
};


enum wiphy_flags {
	WIPHY_FLAG_CUSTOM_REGULATORY	= BIT(0),
	WIPHY_FLAG_STRICT_REGULATORY	= BIT(1),
	WIPHY_FLAG_DISABLE_BEACON_HINTS	= BIT(2),
	WIPHY_FLAG_NETNS_OK		= BIT(3),
	WIPHY_FLAG_PS_ON_BY_DEFAULT	= BIT(4),
	WIPHY_FLAG_4ADDR_AP		= BIT(5),
	WIPHY_FLAG_4ADDR_STATION	= BIT(6),
};

struct mac_address {
	u8 addr[ETH_ALEN];
};

struct wiphy {
	/* assign these fields before you register the wiphy */

	/* permanent MAC address(es) */
	u8 perm_addr[ETH_ALEN];
	u8 addr_mask[ETH_ALEN];

	u16 n_addresses;
	struct mac_address *addresses;

	/* Supported interface modes, OR together BIT(NL80211_IFTYPE_...) */
	u16 interface_modes;

	u32 flags;

	enum cfg80211_signal_type signal_type;

	int bss_priv_size;
	u8 max_scan_ssids;
	u16 max_scan_ie_len;

	int n_cipher_suites;
	const u32 *cipher_suites;

	u8 retry_short;
	u8 retry_long;
	u32 frag_threshold;
	u32 rts_threshold;
	u8 coverage_class;

	char fw_version[ETHTOOL_BUSINFO_LEN];
	u32 hw_version;

	u8 max_num_pmkids;

	/* If multiple wiphys are registered and you're handed e.g.
	 * a regular netdev with assigned ieee80211_ptr, you won't
	 * know whether it points to a wiphy your driver has registered
	 * or not. Assign this to something global to your driver to
	 * help determine whether you own this wiphy or not. */
	const void *privid;

	struct ieee80211_supported_band *bands[IEEE80211_NUM_BANDS];

	/* Lets us get back the wiphy on the callback */
	int (*reg_notifier)(struct wiphy *wiphy,
			    struct regulatory_request *request);

	/* fields below are read-only, assigned by cfg80211 */

	const struct ieee80211_regdomain *regd;

	/* the item in /sys/class/ieee80211/ points to this,
	 * you need use set_wiphy_dev() (see below) */
	struct device dev;

	/* dir in debugfs: ieee80211/<wiphyname> */
	struct dentry *debugfsdir;

#ifdef CONFIG_NET_NS
	/* the network namespace this phy lives in currently */
	struct net *_net;
#endif

#ifdef CONFIG_CFG80211_WEXT
	const struct iw_handler_def *wext;
#endif

	char priv[0] __attribute__((__aligned__(NETDEV_ALIGN)));
};

#ifdef CONFIG_NET_NS
static inline struct net *wiphy_net(struct wiphy *wiphy)
{
	return wiphy->_net;
}

static inline void wiphy_net_set(struct wiphy *wiphy, struct net *net)
{
	wiphy->_net = net;
}
#else
static inline struct net *wiphy_net(struct wiphy *wiphy)
{
	return &init_net;
}

static inline void wiphy_net_set(struct wiphy *wiphy, struct net *net)
{
}
#endif

static inline void *wiphy_priv(struct wiphy *wiphy)
{
	BUG_ON(!wiphy);
	return &wiphy->priv;
}

static inline struct wiphy *priv_to_wiphy(void *priv)
{
	BUG_ON(!priv);
	return container_of(priv, struct wiphy, priv);
}

static inline void set_wiphy_dev(struct wiphy *wiphy, struct device *dev)
{
	wiphy->dev.parent = dev;
}

static inline struct device *wiphy_dev(struct wiphy *wiphy)
{
	return wiphy->dev.parent;
}

static inline const char *wiphy_name(struct wiphy *wiphy)
{
	return dev_name(&wiphy->dev);
}

struct wiphy *wiphy_new(const struct cfg80211_ops *ops, int sizeof_priv);

extern int wiphy_register(struct wiphy *wiphy);

extern void wiphy_unregister(struct wiphy *wiphy);

extern void wiphy_free(struct wiphy *wiphy);

/* internal structs */
struct cfg80211_conn;
struct cfg80211_internal_bss;
struct cfg80211_cached_keys;

#define MAX_AUTH_BSSES		4

struct wireless_dev {
	struct wiphy *wiphy;
	enum nl80211_iftype iftype;

	/* the remainder of this struct should be private to cfg80211 */
	struct list_head list;
	struct net_device *netdev;

	struct list_head action_registrations;
	spinlock_t action_registrations_lock;

	struct mutex mtx;

	struct work_struct cleanup_work;

	bool use_4addr;

	/* currently used for IBSS and SME - might be rearranged later */
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 ssid_len;
	enum {
		CFG80211_SME_IDLE,
		CFG80211_SME_CONNECTING,
		CFG80211_SME_CONNECTED,
	} sme_state;
	struct cfg80211_conn *conn;
	struct cfg80211_cached_keys *connect_keys;

	struct list_head event_list;
	spinlock_t event_lock;

	struct cfg80211_internal_bss *authtry_bsses[MAX_AUTH_BSSES];
	struct cfg80211_internal_bss *auth_bsses[MAX_AUTH_BSSES];
	struct cfg80211_internal_bss *current_bss; /* associated / joined */
	struct ieee80211_channel *channel;

	bool ps;
	int ps_timeout;

#ifdef CONFIG_CFG80211_WEXT
	/* wext data */
	struct {
		struct cfg80211_ibss_params ibss;
		struct cfg80211_connect_params connect;
		struct cfg80211_cached_keys *keys;
		u8 *ie;
		size_t ie_len;
		u8 bssid[ETH_ALEN], prev_bssid[ETH_ALEN];
		u8 ssid[IEEE80211_MAX_SSID_LEN];
		s8 default_key, default_mgmt_key;
		bool prev_bssid_valid;
	} wext;
#endif
};

static inline void *wdev_priv(struct wireless_dev *wdev)
{
	BUG_ON(!wdev);
	return wiphy_priv(wdev->wiphy);
}


extern int ieee80211_channel_to_frequency(int chan);

extern int ieee80211_frequency_to_channel(int freq);

extern struct ieee80211_channel *__ieee80211_get_channel(struct wiphy *wiphy,
							 int freq);
static inline struct ieee80211_channel *
ieee80211_get_channel(struct wiphy *wiphy, int freq)
{
	return __ieee80211_get_channel(wiphy, freq);
}

struct ieee80211_rate *
ieee80211_get_response_rate(struct ieee80211_supported_band *sband,
			    u32 basic_rates, int bitrate);


struct radiotap_align_size {
	uint8_t align:4, size:4;
};

struct ieee80211_radiotap_namespace {
	const struct radiotap_align_size *align_size;
	int n_bits;
	uint32_t oui;
	uint8_t subns;
};

struct ieee80211_radiotap_vendor_namespaces {
	const struct ieee80211_radiotap_namespace *ns;
	int n_ns;
};


struct ieee80211_radiotap_iterator {
	struct ieee80211_radiotap_header *_rtheader;
	const struct ieee80211_radiotap_vendor_namespaces *_vns;
	const struct ieee80211_radiotap_namespace *current_namespace;

	unsigned char *_arg, *_next_ns_data;
	__le32 *_next_bitmap;

	unsigned char *this_arg;
	int this_arg_index;
	int this_arg_size;

	int is_radiotap_ns;

	int _max_length;
	int _arg_index;
	uint32_t _bitmap_shifter;
	int _reset_on_ext;
};

extern int ieee80211_radiotap_iterator_init(
	struct ieee80211_radiotap_iterator *iterator,
	struct ieee80211_radiotap_header *radiotap_header,
	int max_length, const struct ieee80211_radiotap_vendor_namespaces *vns);

extern int ieee80211_radiotap_iterator_next(
	struct ieee80211_radiotap_iterator *iterator);


extern const unsigned char rfc1042_header[6];
extern const unsigned char bridge_tunnel_header[6];

unsigned int ieee80211_get_hdrlen_from_skb(const struct sk_buff *skb);

unsigned int ieee80211_hdrlen(__le16 fc);

int ieee80211_data_to_8023(struct sk_buff *skb, const u8 *addr,
			   enum nl80211_iftype iftype);

int ieee80211_data_from_8023(struct sk_buff *skb, const u8 *addr,
			     enum nl80211_iftype iftype, u8 *bssid, bool qos);

void ieee80211_amsdu_to_8023s(struct sk_buff *skb, struct sk_buff_head *list,
			      const u8 *addr, enum nl80211_iftype iftype,
			      const unsigned int extra_headroom);

unsigned int cfg80211_classify8021d(struct sk_buff *skb);

const u8 *cfg80211_find_ie(u8 eid, const u8 *ies, int len);


extern int regulatory_hint(struct wiphy *wiphy, const char *alpha2);

extern void wiphy_apply_custom_regulatory(
	struct wiphy *wiphy,
	const struct ieee80211_regdomain *regd);

extern int freq_reg_info(struct wiphy *wiphy,
			 u32 center_freq,
			 u32 desired_bw_khz,
			 const struct ieee80211_reg_rule **reg_rule);

int cfg80211_wext_giwname(struct net_device *dev,
			  struct iw_request_info *info,
			  char *name, char *extra);
int cfg80211_wext_siwmode(struct net_device *dev, struct iw_request_info *info,
			  u32 *mode, char *extra);
int cfg80211_wext_giwmode(struct net_device *dev, struct iw_request_info *info,
			  u32 *mode, char *extra);
int cfg80211_wext_siwscan(struct net_device *dev,
			  struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra);
int cfg80211_wext_giwscan(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_point *data, char *extra);
int cfg80211_wext_siwmlme(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_point *data, char *extra);
int cfg80211_wext_giwrange(struct net_device *dev,
			   struct iw_request_info *info,
			   struct iw_point *data, char *extra);
int cfg80211_wext_siwgenie(struct net_device *dev,
			   struct iw_request_info *info,
			   struct iw_point *data, char *extra);
int cfg80211_wext_siwauth(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_param *data, char *extra);
int cfg80211_wext_giwauth(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_param *data, char *extra);

int cfg80211_wext_siwfreq(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_freq *freq, char *extra);
int cfg80211_wext_giwfreq(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_freq *freq, char *extra);
int cfg80211_wext_siwessid(struct net_device *dev,
			   struct iw_request_info *info,
			   struct iw_point *data, char *ssid);
int cfg80211_wext_giwessid(struct net_device *dev,
			   struct iw_request_info *info,
			   struct iw_point *data, char *ssid);
int cfg80211_wext_siwrate(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_param *rate, char *extra);
int cfg80211_wext_giwrate(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_param *rate, char *extra);

int cfg80211_wext_siwrts(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_param *rts, char *extra);
int cfg80211_wext_giwrts(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_param *rts, char *extra);
int cfg80211_wext_siwfrag(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_param *frag, char *extra);
int cfg80211_wext_giwfrag(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_param *frag, char *extra);
int cfg80211_wext_siwretry(struct net_device *dev,
			   struct iw_request_info *info,
			   struct iw_param *retry, char *extra);
int cfg80211_wext_giwretry(struct net_device *dev,
			   struct iw_request_info *info,
			   struct iw_param *retry, char *extra);
int cfg80211_wext_siwencodeext(struct net_device *dev,
			       struct iw_request_info *info,
			       struct iw_point *erq, char *extra);
int cfg80211_wext_siwencode(struct net_device *dev,
			    struct iw_request_info *info,
			    struct iw_point *erq, char *keybuf);
int cfg80211_wext_giwencode(struct net_device *dev,
			    struct iw_request_info *info,
			    struct iw_point *erq, char *keybuf);
int cfg80211_wext_siwtxpower(struct net_device *dev,
			     struct iw_request_info *info,
			     union iwreq_data *data, char *keybuf);
int cfg80211_wext_giwtxpower(struct net_device *dev,
			     struct iw_request_info *info,
			     union iwreq_data *data, char *keybuf);
struct iw_statistics *cfg80211_wireless_stats(struct net_device *dev);

int cfg80211_wext_siwpower(struct net_device *dev,
			   struct iw_request_info *info,
			   struct iw_param *wrq, char *extra);
int cfg80211_wext_giwpower(struct net_device *dev,
			   struct iw_request_info *info,
			   struct iw_param *wrq, char *extra);

int cfg80211_wext_siwap(struct net_device *dev,
			struct iw_request_info *info,
			struct sockaddr *ap_addr, char *extra);
int cfg80211_wext_giwap(struct net_device *dev,
			struct iw_request_info *info,
			struct sockaddr *ap_addr, char *extra);


void cfg80211_scan_done(struct cfg80211_scan_request *request, bool aborted);

struct cfg80211_bss*
cfg80211_inform_bss_frame(struct wiphy *wiphy,
			  struct ieee80211_channel *channel,
			  struct ieee80211_mgmt *mgmt, size_t len,
			  s32 signal, gfp_t gfp);

struct cfg80211_bss*
cfg80211_inform_bss(struct wiphy *wiphy,
		    struct ieee80211_channel *channel,
		    const u8 *bssid,
		    u64 timestamp, u16 capability, u16 beacon_interval,
		    const u8 *ie, size_t ielen,
		    s32 signal, gfp_t gfp);

struct cfg80211_bss *cfg80211_get_bss(struct wiphy *wiphy,
				      struct ieee80211_channel *channel,
				      const u8 *bssid,
				      const u8 *ssid, size_t ssid_len,
				      u16 capa_mask, u16 capa_val);
static inline struct cfg80211_bss *
cfg80211_get_ibss(struct wiphy *wiphy,
		  struct ieee80211_channel *channel,
		  const u8 *ssid, size_t ssid_len)
{
	return cfg80211_get_bss(wiphy, channel, NULL, ssid, ssid_len,
				WLAN_CAPABILITY_IBSS, WLAN_CAPABILITY_IBSS);
}

struct cfg80211_bss *cfg80211_get_mesh(struct wiphy *wiphy,
				       struct ieee80211_channel *channel,
				       const u8 *meshid, size_t meshidlen,
				       const u8 *meshcfg);
void cfg80211_put_bss(struct cfg80211_bss *bss);

void cfg80211_unlink_bss(struct wiphy *wiphy, struct cfg80211_bss *bss);

void cfg80211_send_rx_auth(struct net_device *dev, const u8 *buf, size_t len);

void cfg80211_send_auth_timeout(struct net_device *dev, const u8 *addr);

void __cfg80211_auth_canceled(struct net_device *dev, const u8 *addr);

void cfg80211_send_rx_assoc(struct net_device *dev, const u8 *buf, size_t len);

void cfg80211_send_assoc_timeout(struct net_device *dev, const u8 *addr);

void cfg80211_send_deauth(struct net_device *dev, const u8 *buf, size_t len);

void __cfg80211_send_deauth(struct net_device *dev, const u8 *buf, size_t len);

void cfg80211_send_disassoc(struct net_device *dev, const u8 *buf, size_t len);

void __cfg80211_send_disassoc(struct net_device *dev, const u8 *buf,
	size_t len);

void cfg80211_michael_mic_failure(struct net_device *dev, const u8 *addr,
				  enum nl80211_key_type key_type, int key_id,
				  const u8 *tsc, gfp_t gfp);

void cfg80211_ibss_joined(struct net_device *dev, const u8 *bssid, gfp_t gfp);

void wiphy_rfkill_set_hw_state(struct wiphy *wiphy, bool blocked);

void wiphy_rfkill_start_polling(struct wiphy *wiphy);

void wiphy_rfkill_stop_polling(struct wiphy *wiphy);

#ifdef CONFIG_NL80211_TESTMODE
struct sk_buff *cfg80211_testmode_alloc_reply_skb(struct wiphy *wiphy,
						  int approxlen);

int cfg80211_testmode_reply(struct sk_buff *skb);

struct sk_buff *cfg80211_testmode_alloc_event_skb(struct wiphy *wiphy,
						  int approxlen, gfp_t gfp);

void cfg80211_testmode_event(struct sk_buff *skb, gfp_t gfp);

#define CFG80211_TESTMODE_CMD(cmd)	.testmode_cmd = (cmd),
#else
#define CFG80211_TESTMODE_CMD(cmd)
#endif

void cfg80211_connect_result(struct net_device *dev, const u8 *bssid,
			     const u8 *req_ie, size_t req_ie_len,
			     const u8 *resp_ie, size_t resp_ie_len,
			     u16 status, gfp_t gfp);

void cfg80211_roamed(struct net_device *dev, const u8 *bssid,
		     const u8 *req_ie, size_t req_ie_len,
		     const u8 *resp_ie, size_t resp_ie_len, gfp_t gfp);

void cfg80211_disconnected(struct net_device *dev, u16 reason,
			   u8 *ie, size_t ie_len, gfp_t gfp);

void cfg80211_ready_on_channel(struct net_device *dev, u64 cookie,
			       struct ieee80211_channel *chan,
			       enum nl80211_channel_type channel_type,
			       unsigned int duration, gfp_t gfp);

void cfg80211_remain_on_channel_expired(struct net_device *dev,
					u64 cookie,
					struct ieee80211_channel *chan,
					enum nl80211_channel_type channel_type,
					gfp_t gfp);


void cfg80211_new_sta(struct net_device *dev, const u8 *mac_addr,
		      struct station_info *sinfo, gfp_t gfp);

bool cfg80211_rx_action(struct net_device *dev, int freq, const u8 *buf,
			size_t len, gfp_t gfp);

void cfg80211_action_tx_status(struct net_device *dev, u64 cookie,
			       const u8 *buf, size_t len, bool ack, gfp_t gfp);


void cfg80211_cqm_rssi_notify(struct net_device *dev,
			      enum nl80211_cqm_rssi_threshold_event rssi_event,
			      gfp_t gfp);

#endif /* __NET_CFG80211_H */
